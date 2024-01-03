# Intro

This Romi beginner program will help you understand the basics of programming a Romi robot. A Romi is a small and inexpensive two-wheeled robot, used for educational purposes. WPILib maintains [basic instructions](https://docs.wpilib.org/en/stable/docs/romi-robot/index.html) to get started with programming a Romi. This program will be specific to Team 9450's training program. 

This program is in Java. We suggest you have a basic understanding of how to navigate an IDE, Java coding basics, and how to push code from your computer to the Romi.

There are a few options for a coding environment: [VSCode](https://code.visualstudio.com/download), [IntelliJ](https://www.jetbrains.com/idea/download/), and [Eclipse](https://www.eclipse.org/downloads/packages/installer).

[Getting started with VSCode](https://code.visualstudio.com/docs)

[Getting started with IntelliJ](https://www.jetbrains.com/help/idea/getting-started.html)

[Getting started with Eclipse](https://help.eclipse.org/2023-12/index.jsp?nav=%2F0)

Some other things that are important:

[Learn Java basics](https://www.w3schools.com/java/)

[How to push code to a remote repository using Git](https://docs.github.com/en/get-started/using-git/pushing-commits-to-a-remote-repository).

# Getting Started with the Romi

We will you a coding library called WPILIB to code the Romi (you should aready have the library  installed). You will hop through files and learn stuff from comments and links throughout the file.
Before though you will have to know a few things in java:
* variables
* instance methods
* basic knowledge of loops
* methods and parameters
this is the order of which you will learn things:
- 1 * How to set power to motors and creating methods

- 2 * How to get create objects and tranfer methods from one file to another 
    - You will also quickly lear how to run programs on the Romi

- 3 * How to use controllers (will you a xbox library, but should adapt to almost every other controller)

- 4 * PIDS

- 5 * Commands

- 6 * Good habits and organization

go to RomiDrivetrain.java to start the program.























Now this is simple, Here are some good habits you should follow
* all controller stuff go in RobotContainer 
* Don't add your drivetrain as a instice directly into your Robot.java, instead go through your Container

Now your done 
