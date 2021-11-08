## 第三讲
Python图像处理与CNN基础



### 图像处理介绍：
数字图像是一组格式化的数据，存储这对应不同位置上的视觉信息（亮度，不同频率分量）。根据存储的信息不同，可以划分成不同类型的图像，如存储RGB不同通道信息的彩色图像，存储亮度信息的灰度图像，存储色调、饱和度、亮度信息的HSV图像等。
<img width="242" alt="image" src="https://user-images.githubusercontent.com/74605431/140712177-8b76ccad-4d4c-43be-83f6-711850a993f0.png"><img width="242" alt="image" src="https://user-images.githubusercontent.com/74605431/140712187-af927222-adef-4571-9e56-9eb655c4d0b3.png">


图像处理是从现有图像提前出有效信息，或者去除无关信息。图像滤波是一个常见且有效的手段。如高斯滤波的平滑效果与其卷积核尺寸有很大关系：

<img width="415" alt="image" src="https://user-images.githubusercontent.com/74605431/140712914-acb23784-4b2c-4f59-bcd9-6463eab26cc2.png">



物体检测是指在给定的图片上给出要寻找的物体所在的位置以及类别：

<img width="415" alt="image" src="https://user-images.githubusercontent.com/74605431/140713229-42932944-c056-44d9-a0f5-38a3933e0b47.png">


使用OpenCV提供的通用计算机视觉方法库进行模版匹配：

<img width="415" alt="image" src="https://user-images.githubusercontent.com/74605431/140713579-a6345b5d-ab29-4f16-98e0-d74775747a4b.png">



使用CNN检测目标位置：

<img width="415" alt="image" src="https://user-images.githubusercontent.com/74605431/140714239-acc310a7-9418-4351-b679-91ce07da4948.png">



YOLO将检测变为一个回归问题，从输入的图像仅经过一个神经网络得到bounding box及其所属类别的概率：

<img width="815" alt="image" src="https://user-images.githubusercontent.com/74605431/140714730-d5ef6193-3c0d-4224-af8f-83b066c9a318.png">





### 课程视频：
https://cloud.tsinghua.edu.cn/d/9cefd47098344ef3912d/
