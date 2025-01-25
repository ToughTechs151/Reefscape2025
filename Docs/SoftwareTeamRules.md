# Software Rules

## Software Team Rules

1. If it isn't on Github, it doesn't exist.
2. You are not just coding for yourself, you are also coding for the Robot, the Drive Team, and your future self.
   The code needs to work for the robot, what is in the log or on the Drive Station needs to make sense without looking at the code, and the code needs to make sense when you look at it a year from now. If the first reaction of
   someone else looking at it is "huh?", then you aren't finished.
3. No value in the robot that will be changed/tuned should be hardcoded. Always allow them to be changed from the Drive Station. The best way to do that is to use Preferences. Always log the value used in the log file.
4. Create a Github issue for your current task at hand, whatever it is, then create a branch for the issue and code using the branch. Always commit to this branch before you deploy code to the robot. Record in the
   commit comments on what you are trying to do and why you made your latest change.
5. Make sure Git will record your name in the commits.
6. Never trust setting in firmware. Always restore to factory defaults at robot init, and then setting everything yourself.
7. Whenever reasonable, a class should implement the Sendable interface.

## Rules for other teams working with software

1. The Mechanical and Electrical teams don't build anything until they tell the software team what port everything is plugged into, what PDP it gets power from, the type of every motor, the gear ratio on every gearbox,
   the size and type of all the wheels, the ticks per unit of every encoder. Those teams are free to change them at any time, but you must inform software *before* the change. Feel free to let Software choose a value,
   but we must be informed of what choices are to be made. There should be one official golden source recording this information, preferably on Github.
1. The Drive Team should be the final arbiter of controller assignments. Again, there should be one official source of this information, preferably on Github. Software and the Drive Team and Strategy teams should discuss
   these assignments frequently.

## Software information learned

1. In command-based WPI programming, a subsystem is a mutual exclusion primitive for command scheduling. It’s commonly used to prevent two commands running simultaneously that touch the same hardware, but more generally it’s just categorizing commands that are incompatible with each other. [Chief Delphi](https://www.chiefdelphi.com/t/commandbase-subsystembase-depreciation/438975/7)
1. NEO encoders:
   * Lower resolution.
   * 112 ms filter delay, not adjustable, versus 82ms default, adjustable on the AMT. [Chief Delphi](https://www.chiefdelphi.com/t/thoughts-on-neos-and-spark-max/400769/65?u=blu28). This is only on velocity measurements. Position is not delayed.
   * Wrong end of the backlash.
   * Comes for free with brushless motors.
