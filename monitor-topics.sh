#!/bin/bash

# Loop to run every second
while true; do
    # Get a list of all topics
    topics=$(ros2 topic list)
    
    # Filter topics starting with the required prefixes
    filtered_topics=$(echo "$topics" | grep -E "^/arenacam|^/oxts|^/rslidar_|^/cam")

    # Get the count of filtered topics
    topic_count=$(echo "$filtered_topics" | wc -l)
    
    # Print the count (excluding /tf, /tf_static)
    echo "Number of sensor topics being published to: $topic_count"
    
    # Sleep for 1 second before repeating
    sleep 1
done

