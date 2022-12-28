#include <stdlib.h>
#include <BT.h>

bt_status_t Fallback_tick(struct bt_node const * const this)
{
    int i = 0;
    bt_status_t node_status = BT_STATUS_FAILURE;

    struct bt_node const *child = this->child[i];
    while(child != NULL){
        node_status = (*child->tick)(child);
        if(node_status != BT_STATUS_FAILURE)
            return node_status;
        else
            child = this->child[++i];
    }
    
    /* all fallback node children FAILED */
    return BT_STATUS_FAILURE;
}
