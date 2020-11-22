#ifndef SUBSCRIPTION_PUBLISHER_HPP
#define SUBSCRIPTION_PUBLISHER_HPP

#include "ros/ros.h"
#include <functional>
#include <string>

template<typename subtype, typename pubtype>
class SubscriptionPublisher
{

private:

  ros::NodeHandle nodeHandle; 
  ros::Subscriber subscriber;
  ros::Publisher publisher;

  std::function< pubtype(subtype) > function;

public:

  // NOTE: function can be null which will cause no manipulation to take place on subtype.
  //       if no transform is applied to subtype it has to be the same as pubtype to be published!
  SubscriptionPublisher(std::string subtopic, std::string pubtopic, std::function<pubtype(subtype)> function)
  : nodeHandle( ros::NodeHandle() )
  , subscriber(nodeHandle.subscribe(subtopic, 1, &SubscriptionPublisher::callback, this))
  , publisher(nodeHandle.advertise<pubtype>(pubtopic, 1))
  {
      if(function){ this->function = function; } 
  }

  void callback(const subtype& inputMessage)
  {
    if(function)
    {
        publisher.publish( function(inputMessage) );
    }
    else if( std::is_same<pubtype, subtype>::value )
    {
        publisher.publish(inputMessage);
    }   
  }
};

#endif //SUBSCRIPTION_PUBLISHER_HPP