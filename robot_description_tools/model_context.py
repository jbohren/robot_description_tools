

from lxml import etree
from lxml.builder import E

from .util import strcat

class ModelContext:
    """
    Context manager for building URDF and SRDF models.

    This class is used to generate elements for an lxml etree builder, and does
    not maintain any state outside of that which is relevant to the
    construction context.

    Names not starting with '/' will be prefixed with the given string prefix.
    """

    def __init__(self, prefix=None, parent=None):
        """
        prefix: used for name identifiers
        parent: a parent context used for nesting
        """
        self.prefix = prefix or ''
        self.parent = parent

    def resolve(self, name):
        """
        TODO: tf2 disallows frames with initial forward slash
        """
        if len(name) > 0 and name[0] == '/':
            return name[1:]
        else:
            return self.prefix+name

class URDFModelContext(ModelContext):
    def __init__(self, prefix=None, parent=None):
        super().__init__(prefix, parent)

    def material(self, name, rgba):
        return E.material(
                E.color(rgba=strcat(*rgba)),
                name=self.resolve(name))

    def link(self, name, inertia, visual, collision, material):
        return E.link(
                inertia,
                E.visual(*visual, E.material(name=self.resolve(material))),
                E.collision(*collision),
                name=self.resolve(name))

    def virtual_link(self, name):
        return E.link(name=self.resolve(name))

    def fixed_joint(self, name, parent, child, xyz, rpy):
        return E.joint(
                E.parent(link=self.resolve(parent)),
                E.child(link=self.resolve(child)),
                E.origin(
                    xyz=strcat(*xyz),
                    rpy=strcat(*rpy)),
                name=self.resolve(name),
                type='fixed')

    def revolute_joint(self, name, parent, child, xyz, rpy, axis, limit):
        return E.joint(
                E.parent(link=self.resolve(parent)),
                E.child(link=self.resolve(child)),
                E.origin(
                    xyz=strcat(*xyz),
                    rpy=strcat(*rpy)),
                E.axis(
                    xyz=strcat(*axis)),
                limit,
                name=self.resolve(name),
                type='revolute')

    def simple_transmission(self, joint_name):
        return E.transmission(
                E.type('transmission_interface/SimpleTransmission'),
                E.joint(
                    E.hardwareInterface('EffortJointInterface'),
                    name=self.resolve(joint_name)),
                E.actuator(
                    E.mechanicalReduction('1'),
                    name=f'{self.resolve(joint_name)}_actuator'),
                name=f'{self.resolve(joint_name)}_transmission')

class SRDFModelContext(ModelContext):
    def __init__(self, prefix=None, parent=None):
        super().__init__(prefix, parent)

    def chain_group(self, name, base_link, tip_link):
        return E.group(
                E.chain(
                    base_link=self.resolve(base_link),
                    tip_link=self.resolve(tip_link)),
                name=self.resolve(name))

    def group_state(self, name, group, joints):
        return E.group_state(
                *[E.joint(name=self.resolve(name), value=f'{val}')
                    for name, val
                    in joints.items()],
                name=self.resolve(name),
                group=self.resolve(group))

    def end_effector(self, name, group, parent_link):
        return E.end_effector(
                name=self.resolve(name),
                group=self.resolve(group),
                parent_link=self.resolve(parent_link))

    def disable_collisions(self, link1, link2, reason):
        return E.disable_collisions(
                link1=self.resolve(link1),
                link2=self.resolve(link2),
                reason=reason)
