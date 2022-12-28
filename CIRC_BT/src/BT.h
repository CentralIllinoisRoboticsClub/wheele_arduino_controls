#ifndef BT_NODE_H_
#define BT_NODE_H_

#define BT_STATUS_SUCCESS 0
#define BT_STATUS_FAILURE -1
#define BT_STATUS_RUNNING  1
typedef int bt_status_t;

#define BT_NODE_TYPE_SEQUENCE       0
#define BT_NODE_TYPE_FALLBACK       1
#define BT_NODE_TYPE_DECORATOR      2
#define BT_NODE_TYPE_LEAF           3 /* action or condition */
#define BT_NODE_TYPE_SEQUENCE_STAR  4

typedef unsigned int node_type_t;

struct bt_node
{
    node_type_t type;
    bt_status_t (*tick)(struct bt_node const * const);
    const struct bt_node** child;
};


#endif
