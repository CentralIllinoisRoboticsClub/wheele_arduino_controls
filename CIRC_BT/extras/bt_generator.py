import xml.etree.ElementTree as ET
import sys,getopt

class BTNode:
    def __init__(self,node_id,node_type,tick,child_list_id,child_list):
        self._id = node_id
        self._type = node_type
        self._tick = tick
        self._child_list_id = child_list_id
        self._child_list = child_list

    @property
    def id(self):
        return self._id

    @property
    def type(self):
        return self._type

    @property
    def tick(self):
        return self._tick

    @property
    def child_list_id(self):
        return self._child_list_id

    @property
    def child_list(self):
        return self._child_list

class BTDataGenerator:
    def __init__(self,infile,outfile):
        xml = ET.parse(infile)
        xml_root = xml.getroot()
        self.bt_root = xml_root.find('BehaviorTree')
        self.subtree = dict()
        trees = xml_root.findall('BehaviorTree')
        for tree in trees:
            self.subtree[tree.get("ID")] = tree

        self.node_index = 0
        self.outfile = outfile

        # initialize an empty set that will hold unique nodes that
        # are part of the behavior tree
        self.node_ids = set()
        self.nodes = set()

    def generate_node(self,node):

        child_list = []
        for child in node:
            child_id = self.generate_node(child)
            child_list.append('&' + child_id +',\n')

        if node.tag == 'Fallback':
            node_id = node.tag + str(self.node_index)
            node_type = 'BT_NODE_TYPE_FALLBACK'
            tick = 'Fallback_tick'
        elif node.tag == 'Sequence':
            node_id = node.tag + str(self.node_index)
            node_type = 'BT_NODE_TYPE_SEQUENCE'
            tick = 'Sequence_tick'
        elif node.tag == 'SequenceStar':
            node_id = node.tag + str(self.node_index)
            node_type = 'BT_NODE_TYPE_SEQUENCE_STAR'
            tick = 'Sequence_tick'
        elif node.tag == 'Decorator':
            node_id = node.tag
            node_type = 'BT_NODE_TYPE_DECORATOR'
            tick = node.tag + '_tick'
        elif node.tag == 'Condition':
            node_id = node.get('ID')
            node_type = 'BT_NODE_TYPE_LEAF'
            tick = node_id + '_tick'
        elif node.tag == 'Action':
            node_id = node.get('ID')
            node_type = 'BT_NODE_TYPE_LEAF'
            tick = node_id + '_tick'
        elif node.tag == 'SubTree':
            node_id = node.get('ID')
            node_type = 'BT_NODE_TYPE_SUBTREE'
            tick = 'Subtree_tick'
            subtree_root = self.subtree[node_id]
            child_id = self.generate_node(subtree_root[0])
            child_list.append('&' + child_id +',\n')
        else:
            node_id = node.tag
            node_type = 'BT_NODE_TYPE_LEAF'
            tick = node.tag + '_tick'

        child_list.append('NULL\n')

        if(len(node) > 0):
            child_list_id = node_id + '_childlist'
            self.node_index += 1
        elif node.tag == 'SubTree':
            child_list_id = node_id + '_childlist'
            self.node_index += 1
        else:
            child_list_id = 'NULL'

        # check if node is already a member of the set of nodes
        # that are part of this behavior tree
        # (see Python "set" datatype: https://docs.python.org/3/library/stdtypes.html#set)
        if node_id not in self.node_ids:
            self.node_ids.add(node_id)
            Node = BTNode(node_id,node_type,tick,child_list_id,child_list)
            self.nodes.add(Node)

        return node_id

    def codegen(self):
        self.f = open(self.outfile, 'w')
        self.f.write('#include <stdlib.h>\n')
        self.f.write('#include <BT.h>\n\n')
        # pass behavior tree root node to code generator
        root_node_id = self.generate_node(self.bt_root[0])

        # forward declarations
        for node_id in self.node_ids:
            self.f.write('static const struct bt_node ' + node_id + ';\n')
        self.f.write('\n')

        for node in self.nodes:
            self.f.write('extern bt_status_t ' + node.tick + '(struct bt_node const * const this);\n')
        self.f.write('\n')

        for node in self.nodes:
            child_list = node.child_list
            if len(child_list) > 1:
                self.f.write('static const struct bt_node *' + node.id + '_childlist[] =\n')
                self.f.write('{\n')
                for child_ptr in child_list:
                    self.f.write('   ' + child_ptr)
                self.f.write('};\n\n')

            self.f.write('static const struct bt_node ' + node.id + ' =\n')
            self.f.write('{\n')
            self.f.write('   ' + node.type + ',\n')
            self.f.write('   &' + node.tick + ',\n')
            self.f.write('   ' + node.child_list_id + ',\n')
            self.f.write('};\n\n')

        self.f.write('const struct bt_node *bt_root = &' + root_node_id + ';\n')
        self.f.close()


if __name__=="__main__":
    argv = sys.argv[1:]
    input_tree = argv[0]
    outfile = argv[1]
    BT = BTDataGenerator(input_tree,outfile)
    BT.codegen()
