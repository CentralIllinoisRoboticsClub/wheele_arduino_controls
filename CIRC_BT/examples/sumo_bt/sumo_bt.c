#include <stdlib.h>
#include <BT.h>

static const struct bt_node ClockwiseSpiral;
static const struct bt_node Fallback0;
static const struct bt_node FaceOpponent;
static const struct bt_node DriveForward;
static const struct bt_node StartDelayFinished;
static const struct bt_node OpponentNear;
static const struct bt_node SequenceStar1;

extern bt_status_t StartDelayFinished_tick(struct bt_node const * const this);
extern bt_status_t FaceOpponent_tick(struct bt_node const * const this);
extern bt_status_t OpponentNear_tick(struct bt_node const * const this);
extern bt_status_t Fallback_tick(struct bt_node const * const this);
extern bt_status_t ClockwiseSpiral_tick(struct bt_node const * const this);
extern bt_status_t Sequence_tick(struct bt_node const * const this);
extern bt_status_t DriveForward_tick(struct bt_node const * const this);

static const struct bt_node StartDelayFinished =
{
   BT_NODE_TYPE_LEAF,
   &StartDelayFinished_tick,
   NULL,
};

static const struct bt_node FaceOpponent =
{
   BT_NODE_TYPE_LEAF,
   &FaceOpponent_tick,
   NULL,
};

static const struct bt_node OpponentNear =
{
   BT_NODE_TYPE_LEAF,
   &OpponentNear_tick,
   NULL,
};

static const struct bt_node *Fallback0_childlist[] =
{
   &OpponentNear,
   &ClockwiseSpiral,
   NULL
};

static const struct bt_node Fallback0 =
{
   BT_NODE_TYPE_FALLBACK,
   &Fallback_tick,
   Fallback0_childlist,
};

static const struct bt_node ClockwiseSpiral =
{
   BT_NODE_TYPE_LEAF,
   &ClockwiseSpiral_tick,
   NULL,
};

static const struct bt_node *SequenceStar1_childlist[] =
{
   &StartDelayFinished,
   &Fallback0,
   &FaceOpponent,
   &DriveForward,
   NULL
};

static const struct bt_node SequenceStar1 =
{
   BT_NODE_TYPE_SEQUENCE_STAR,
   &Sequence_tick,
   SequenceStar1_childlist,
};

static const struct bt_node DriveForward =
{
   BT_NODE_TYPE_LEAF,
   &DriveForward_tick,
   NULL,
};

const struct bt_node *bt_root = &SequenceStar1;
