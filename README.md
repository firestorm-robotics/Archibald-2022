# Archibald - 2022

The FRC robot for our team (Firestorm Robotics, FRC 6341). This is a code repository, and most of our stuff will be in Gitbook, however, I would like to make a few things clear for new coders, in this file.

* Use your own branch! Ask me and I'll make one for you. That way you can develop without corrupting our master branch.
* Comment your code rigorously but not superfluously. Explain what complicated lines do. `return (static const double*)x::c -> lb << 2 + 7` should have the comment `// C in the namespace x is a pointer with an lb, which we bitshift by 2 and add seven to, then convert to a static const double and return`, but `float n` doesn't need an explanation.
* If you have more than one variable to contain data on something, it needs a wrapper class. This isn't me being an OOP stickler, it's just clean code. We don't need this to be unexplainably junky.
* Pay attention to thread safety, because if you don't the robot could crash. I try to keep threading away from our code but there are points at which we can't avoid it. I'll do my best to flag functions as thread-safe or thread-unsafe.
* If your classes can be grouped into individual semi-self-contained sections, it's time to create another file. Proper conventions tell us to go to the Fiery Chaos and back to do this, but they can go stick their heads in a pig. Use .hpp files and include directives.
* Don't write your own stuff until you know you can't use somebody else's. Actually this is a lie, totally write your own libraries, just make sure they aren't buggy.
* Keep your code as self-contained as possible so you can unit-test without a robot.