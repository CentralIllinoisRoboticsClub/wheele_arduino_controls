# CIRC Behavior Tree Library (CIRC_BT)

This library provides a basic behavior tree framework for small embedded systems. Use it with Arduino projects, or outside Arduino with a little bit of know how.

## Usage
Design a behavior tree using Groot or similar, and export the XML representation of the tree. Then use the python generator script (included in the library extras folder) to translate the XML to data structures. The library implements basic behavior tree nodes such as sequence, fallback, etc. (see below). The generator only creates the structural framework; implementation details must be provided by the user for custom action, condition, and decorator nodes.


## Built-In Node Support

* Sequence
* Sequence with Memory (SequenceStar)
* Fallback
* AlwaysFailure
* AlwaysSuccess

## Examples
Load the sumo_bt example in the Arduino IDE for additional usage details
