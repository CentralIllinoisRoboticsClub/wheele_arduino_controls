#include <BT.h>

bt_status_t AlwaysSuccess_tick(struct bt_node* this)
{
    (void)this;
    return BT_STATUS_SUCCESS;
}
