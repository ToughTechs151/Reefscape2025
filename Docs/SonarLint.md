# Handling SonarLint Flagged Problems

## Member Field and Constant Name Formatting Errors

The Google style rules that we use differ in a couple of key ways from the style used in the WPILib. These are the way instance and class fields are named and the way constants are named.

### Member fields

The Google style is to not treat these field names any differently from any other field name. It specifies what is known as "Camel Case". This format starts with a lower case letter and then using a single upper case letter at the start of each new word in the sequence, so that a field might look something like "camelCaseHasWordsTogether". The WPI style uses this same kind of pattern, except that member fields have the prefix "m_" in front of the name. Since objects are often passed into a class in a constructor, it would seem logical to use the same name. To differentiate between a parameter and a member of the same name, a common style is to put the "m_" in front. However, this style has recently fallen out of favor and instead, it is considered better to use the "this." construct that Java has built in to resolve the ambiguity.

Because we often import example code from WPI, the checkstyle extension has been modified to ignore the problem if there is a "m_" before a field. We do not have a way to modify Sonarlint in advance like that, but we can suppress the warnings on a case by case basis.

### Constant names

Constants in Java are defined by having the attributes of being final and static. The naming convention for constants is to use all capital letters with different words separated by an underline ("_") character. The WPI convention is to use a mixed case name prefixed with s single lower case "k" to indicate a constant.

### Suppressing Warnings in Java

Java has a general mechanism for telling the system to suppress flagging certain types of errors. It does this using the Java Annotation "@SuppressWarnings". You are probably already familiar with the @Override annotation, this works in the same way.

Anytime you get a SonarLint error the problem will be highlighted with a yellow squiggly line. If you place the cursor on the yellow line, it will pop up a window that will explain what the problem is. This window will also have two options at the bottom, "View Problem" and "Quick Fix". The view problem option will give you a longer explanation of the issue. The quick fix option will present ways you can deal with the issue. Sometimes the quick fix will be a real fix, but for these naming issues there isn't much to offer except to suggest changing the name. It will also give the option of disabling a particular rule, but do not choose this since it disables it at the user level, and not just for the current project. The pop up window will also list the SonarLint Rule number, which will look something like "java:S116". You will need this string to suppress the warning.

The best solution is to actually change the nme to conform to our style, but sometimes there can be many errors and you might want to wait to do that. To suppress the warning on an instance by instance basis, you can add a new line just before the line in question, and add a line like this:

```code
@java.lang.SuppressWarnings("java:S116")
```

Note: the "java"lang" is not always needed, but best to include them.

The line above will tell SonarLint to ignore rule S116 for that next line, just like the @Override tells Java that the method on the next line should override the method in the Super class. At sometime in the future you should go back and rename the fields to a conforming value.

### Suppressing Warnings for a class

If the code you are importing has many instances of the same kind of warning, you can put the SuppressWarnings line on the line before the class name and it will suppress the warnings for the entire class. This is handy, but remember that it will suppress the warning everywhere in the class, so you will not get the warning on new code you write.

### Suppressing the warning with auto completion

The template includes a code snippet called "slname", for (SonarLint Naming errors) to automatically insert the annotation to make it easier without having to remember the exact syntax. The snippet is set to suppress both S115 and S116, the member name and constant name warnings. To use it, create an empty line on the line before the line with the issue and start typing "slname". After you get the "sl" there should be a pop up that lists slname as an option. Simply hit the enter key to accept the snippet. This will also work at the line before a class.

### Un-suppressing the warnings

To reactivate a warning, just delete the SuppressWarnings line and the rule will become reactivated.

### Changing the name of constants

VSCode has a feature to allow you to rename any Java symbol. Place the cursor in the symbol you wish to rename and then right-click and select "rename symbol" or press F2. A dialog will pop up where you can type the name you want and then press enter. This is safe to do with constants because VSCode will change the name where ever it is used in the workspace. So for constants you just change the name to the same name in the proper format. So if you had a constant named "kThisIsAConstant", you would just type "THIS_IS_A_CONSTANT".

### Changing the name of members

Unfortunately, removing the leading "m_" is a little trickier. We can't simply change the name the same way we did for constants because we generally use the m_ when we wanted to use the same name for a parameter and a member. The easiest option is to change the name of the member field to be something unique, then we can use the rename feature in VSCode. But usually we lose the obvious semantic meaning of the name. For instance if a member field is m_driveMotor, we might expect that the constructor has a parameter called driveMotor, so if we just removed the "m_" the two names would collide, but if we changed the name to something else it might not be recognizable to a reader as being the drive motor. If there is no collision, then you can go ahead and just rename the symbol.

If there is a collision, you can resolve the collision by using the "this" construct. There is no easy automatic feature to do this though. The easiest way is to use "find and replace". Under the "Edit" menu, select "Replace" (ctrl-h). This will pop up a dialog with two fields, "find" and "replace". Enter the name of the field in the find line (in our example it would be "m_driveMotor") and in the replace field put the name after the string "this." ("this.driveMotor"). Select "Match case" by clicking on the symbol "Aa" and "Match Whole Word" by clicking on the "ab" that is underlined. It will show you how many matches there are. Each time you press enter, it will replace the current instance and move on to the next. You can click on the down arrow in the dialog to skip one.

After you have changed all the instances, you will probably get an error because you will have a "this." in front of the variable on the line where it is declared, which is illegal. Just delete the "this." from this line.

One thing to be careful of is the variable name matching the beginning of a longer variable name. For instance, "m_leftDrive" will match the beginning of "m_leftDriveEncoder" if you don't have "Match whole word" turned on. There may be cases where something might match even if it is on. So, be careful when you change and look at each case before you change it with the find and replace.
