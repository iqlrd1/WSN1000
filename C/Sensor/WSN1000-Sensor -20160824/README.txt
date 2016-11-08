- based on 20160809 version
1、通过两个mesh-on和mesh-off定时器交叉运行累加到发送定时器时间时，发送一次数据，即通过定时器的方法替代计数器累加   ，功耗方面也较好在6.3uA（睡眠）和30mA（工作）来回切换
2、因第一次运行接收正常，restart/repower-on老是接收不到----改动点：readpersistentstore去除第一次和第二次差异，直   接都去擦除，除了app_nvm_version/sanity_word/currentSensorStatus