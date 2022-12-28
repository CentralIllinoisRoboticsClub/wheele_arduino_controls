#include <BT.h>

// Using the CIRC_BT library:
// 1. Include the CIRC_BT library in your sketch.
// 2. Use Groot to create a behavior tree XML (e,g, sumo_bt.xml for a sumo robot)
// 3. Use python script included with CIRC_BT to translate XML to code structure
//      e.g. python //libraries/CIRC_BT/extras/bt_generator.py sumo_bt.xml sumo_bt.c
//           1st argument is input tree, 2nd argument is output .c file
// 4. Add custom node implementations to sketch_dir/src/bt_nodes/
// 5. Extern behavior tree root node
//      extern const struct bt_node *bt_root;
// 6. Invoke root node tick function from main loop
//      bt_root->tick(bt_root);

extern const struct bt_node *bt_root;

void setup() {
  // put your setup code here, to run once:

}

void loop() {
  // put your main code here, to run repeatedly:
  bt_root->tick(bt_root);

}
