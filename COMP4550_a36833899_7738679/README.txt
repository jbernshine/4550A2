Assignment 3
Chris Greening - 6833899
Justine Bernshine - 7738679

This simulates a pizza cooker that cooks pizzas one at a time, with a user-selectable scheduling algorithm.

On starting the pizza cooker, short press to cycle through the available scheduling algorithms. These are:

 - Yellow LED: fixed priority scheduling
 - Red LED: earliest deadline first
 - Blue LEd: least laxity first

On a long press, the pizzas will cook with the selected algorithm. Note that:

 - If a pizza misses its deadline, pizzas that have not will be prioritized.
 - In case of a tie, pizzas' priorities will determine their priority, even if the selected scheduling algorithm does not normally make use of this (ie. least laxity first).
 - If THIS is a tie, pizzas will cook round robin.

When a pizza completes, a half second buzz will occur, with a frequency particular to that type of pizza.
When a pizza misses its deadline, the led corresponding to that pizza will flash twice.
At the beginning of a pizza's period, it will silently discard any work done on that pizza in the previous period.

Note that the button is disabled when sounds are playing.

Code for the sound was adapted from the sample project.
author: A.Finkelmeyer
modified: MC Lau
original source: http://www.mind-dump.net/configuring-the-stm32f4-discovery-for-audio