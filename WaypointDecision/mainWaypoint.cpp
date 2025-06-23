#include <iostream>

int main(){
    //TODO:
    //Parse input commands or YAML File.
    //The Foramt could be Waypoint -> given seconds. 
    //In the way future, we could make a waypoint seconds generator function.
    //Create a Executive Scheduler based on timestamps and then the waypoint.
        //Think of a Data Strcuture for this.
        //My suggestion, Linked List (if we want to reorganize order) vs vector(if we do not care about reorganize order).
    //Run the Exeuctive Scheduler to publish WayPoints onto the waypoint topic.
    //The topic just to be clear will publish continously the message that contains the waypoint at this current moment.
    //Listen to Position Topic, do some simple reasoning to determine to move to next waypoint.

    //Pseudo code
    /*
        (This can be refactored later)
        Get the file of waypoint commands.
        Go through the file using getline or YAML
        Make the linked list node as we go through it setting the waypoint and the time of seconds.
       Make a function (the location of the func depends on the data implementation structure) that has the time loop in William's timed pwm functionality.

        Make the publisher shared ptr for the Waypoint so that either a linked list or vector impelmentation works.
    */
}