# perspective_filter
Underworlds filter for perspective taking &amp; beliefs computation

This Underworlds client compute the perspectives for a defined set of agent and update the corresponding beliefs worlds

## ROS services

`perspective_filter/add_agent` :
Add an agent to the perspective taking computation

`perspective_filter/remove_agent` :
Remove an agent from the perspective taking computation

## Facts produced

`see(X,Y)`: where X is an agent and Y an object or an agent

`visible(X,Y)`: where X is an object or agent and Y an agent
