# robotnik_log_to_csv

The robotnik_log_to_csv package, based on RComponent structure. Node to save the log topic into a .csv file.

## Installation

(Optional)


## 1 log_to_csv_node

Node to save the log topic into a .csv file.

### 1.1 Parameters

* ~log_topic (String, default: robotnik_hmi/log)
   Name of the log topic subscriber.
   
### 1.2 Subscribed Topics

* robotnik_hmi/log (std_msgs/String)
  Log topic subscriber

### 1.3 Published Topics

* log_to_csv/data (std_msgs/String)
  Information about the node execution

### 1.4 Bringup

```console
roslaunch robotnik_log_to_csv log_to_csv.launch
```
