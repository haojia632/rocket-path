rocket-path
===========

Attempts to learn how to do optimal trajectory planning.

This program was built with Microsoft Visual Studio Community 2015. It uses these libraries:

  * [FreeGLUT] (http://freeglut.sourceforge.net/) for hopefully relatively portable window management
  * [Eigen] (http://eigen.tuxfamily.org/index.php?title=Main_Page) for linear algebra

There are multiple experiments, accessible via the F1-F4 hotkeys. The first one (F1) is an attempt to make a two-cubic trajectory that passes through three points in two dimensions. I don't think it does much at the moment other than plot accelerations and let you move around the points by clicking and dragging them with the mouse.

Problems F2-F4 all simplify things to a one-dimensional problem. The upper plot represents position as a function of time, while the lower plot represents acceleration as a function of time. There are two cubic segments, colored yellow and cyan. The start and end positions are fixed, while the position where the two cubics meet can be moved to change the problem. The velocity at the beginning of the yellow segment is fixed at zero, as is the velocity at the end of the cyan segment. This leaves three free variables: the velocity at the join point and the time durations of the two segments.

The goal is to find the trajectory with the shortest duration that keeps acceleration magnitudes within a limit. Because the trajectory is represented by cubic segments, the acceleration constraints can be enforced at the endpoints.

All of the variables can be directly manipulated via hotkeys, as can the midpoint position:

  * Home/End: Increment/Decrement segment 0 duration
  * PgUp/PgDn: Increment/Decrement segment 1 duration
  * Right/Left: Increment/Decrement midpoint velocity
  * Up/Down: Increment/Decrement midpoint position\n

Each problem prints some help when it's activated.

Version F3 is the most successful. It has eight acceleration constraints; lower and upper bounds on the acceleration at the start and end of each of the two cubic segments. Its additional hotkeys are:

  * I: Reinitialize
  * N: Take a step via interior-point method
  * S: Print current state

Version F4 replaces the eight constraints with four constraints that are squared acceleration magnitude error. It does not work very well; the interior-point solver tends to settle the wrong direction. I don't know how to fix it yet.
