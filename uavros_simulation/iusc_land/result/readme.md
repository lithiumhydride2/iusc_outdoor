## 启动仿真
``` bash
roslaunch iusc_land iusc_outdoor_land.launch num_uav:=6
```
等待终端中显示 `map2local_server init done `

### 控制无人机起飞
``` bash
iuscland 1 takeoff & iuscland 2 takeoff & iuscland 3 takeoff & iuscland 4 takeoff & iuscland 5 takeoff & iuscland 6 takeoff
```
进入offboard
``` bash
iuscland 1 offboard & iuscland 2 offboard & iuscland 3 offboard & iuscland 4 offboard & iuscland 5 offboard & iuscland 6 offboard
```
uav1 进行 formation detect, 等待所有无人机到达指定位置后降落
``` bash
iuscland 1 land & iuscland 2 land & iuscland 3 land & iuscland 4 land & iuscland 5 land & iuscland 6 land
```
无人机降落之后
``` bash
iuscland 1 disarm & iuscland 2 disarm & iuscland 3 disarm & iuscland 4 disarm & iuscland 5 disarm & iuscland 6 disarm
```

### 向实际迁移
uav1 控制其他无人机通过 `land_strategy.py`中下函数实现：
``` python
    def pub_land_pos_index(self):
        pass
        # uav1 控制 uav2 至 uav6
        index_publishers = [
            rospy.Publisher(
                "/uav{}/uav{}/land_index".format(self.uav_id, uav_number),
                Int8,
                latch=True,
                queue_size=10,
            )
            for uav_number in range(1, 7)
        ]

        for _ in range(5):
            for index, index_pub in enumerate(index_publishers):
                # 根据 index 索引目标降落位置
                index_pub.publish(self.land_pos_index[index])
            self.rate1.sleep()

```
需通过通信传递 `index_publishers` 中的话题， 无需改变话题名称