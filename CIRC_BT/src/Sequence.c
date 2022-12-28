#include <stdlib.h>
#include <BT.h>

bt_status_t Sequence_tick(struct bt_node const * const this)
{
    static int current_node = 0;
    bt_status_t node_status = BT_STATUS_FAILURE;
    struct bt_node const * child;

    if(this->type != BT_NODE_TYPE_SEQUENCE_STAR)
        current_node = 0;

    child = this->child[current_node];

    while(child != NULL){
        node_status = (*child->tick)(child);
        if(node_status != BT_STATUS_SUCCESS)
            return node_status;
        else
            child = this->child[++current_node];
    }

    /* all sequence node children succeeded */
    return BT_STATUS_SUCCESS;
}
