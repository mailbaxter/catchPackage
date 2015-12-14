# catchPackage
Baxter mail sorting automation code for course project of EE 106A, Fall 2015

Goal:
We hope to achieve mail sorting automation using the Baxter. In other words, we expect the Baxter to replace an experienced staff in a local mail station for mail sorting jobs. Hence, the Baxter should perform mail catching when someone hands him a mail or a small package, sort the mail according to handwritten zip codes on the surface of the mails, and finally pile up the mails based on size.

Design:
The final design of the mail sorting automation project includes 3 steps. The robot first locates the handed mail. We use AR tags to denote the size of the mails for tracking, and we set up the camera in the Baxter's arm so that it can read information from the AR tags to plan a path and catch the mail. And then the robot puts the mail in the correct region according to the handwritten zip code on the mail. Machine Learning algorithms are adopted to first recognize the contours of the digits and then classify the handwritten digits. The classification model is a Neural Network trained on the MNIST dataset. At last, after all the mails have been sorted, the Baxter piles each heaps so that the order of each pile is well-organized: biggest mails at the bottom and smallest mails on the top. Here we developed a customized algorithm to solve the sorting problem.

Entire report:
http://mailbaxter.github.io/

Illustration video:
https://youtu.be/S1dWw2a7hj8

The code we wrote are in MZP/src/project/src
