# version: 1.2(国赛版本)
# hg_ai_robot.h  头文件
# hg_ai_robot.cpp  调用接口


# 增加力矩监听机制,防止出现严重撞机,由于不同机械臂自身状态不同，可能导致运行状态下电，如有问题请联系技术人员修改

# 增加速度控制,防止出现速度过快出现意外事故，限制最快为0.3m/s,如需要增大请联系技术人员修改

# 增加参数修改限制，防止出现误修改出现撞机，由于机械安装存在误差，如无法通过修改参数实现抓取放置等功能，可能为参数限制，请联系技术人员进行修改

## bjj_grasp.launch: 物料台钣金件单独抓取测试
## bjj_place_demo.launch: 装配台钣金件单独放置
## grasp_test_demo.launch: 物料台物料抓取测试
## new_place_demo.launch: 装配台物料放置测试
## new_product_grasp_demo.launch: 成品抓取
## product_place_demo.launch: 成品放置
## place_grasp_demo.launch: 综合任务
## test_demo.launch: 查看摄像头视野
## get_robot_moment: 获取力矩监听数值(使用请联系技术人员)
## palce_grasp_demo_new.launch: 综合任务升级版


# 注意：
##     未确定调整点位的前提下，请勿将机械臂控制速度调大
##     调试速度建议为0.1;调试完成后速度建议为0.2;
##     test_demo开启了力矩监听防碰撞模式，可以进行左右上下防碰撞（未经技术人员指导请勿轻易尝试）
##     需要开启力矩监听防碰撞功能请联系技术支持

# 更新说明:
##   1.修改立体仓库放置动作,取消旋转90°放置
##   1.修改联调过程,增加无序抓取目标物料
