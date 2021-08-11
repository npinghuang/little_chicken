#!/usr/bin/env python
# coding=utf-8
import rospy
import Tkinter as tk
from Tkinter import *
import tkMessageBox as mb
import ttk
from std_msgs.msg import *
import time
from main2021.msg import *

# Publishers
status_pub = rospy.Publisher('update_status', Int32, queue_size=10)
strategy_pub = rospy.Publisher('strategy', Int32, queue_size=10)

# pv
strategy_vars = ['1', '2', '3', '4']
default_font = 'OptimalC'

def talker():
    rospy.init_node('eurobot_gui', anonymous=True)
    rate = rospy.Rate(10)

class Application(tk.Frame):
    def __init__(self, master):
        
        tk.Frame.__init__(self, master)
        self.pack()
        # In-class variables
        self.current_score = 0
        self.current_script = 0
        self.current_status = 87
        self.error_count = 0
        self.current_time = 0.0
        # Subcribers
        score_sub = rospy.Subscriber('score', Int32, self.score_callback)
        status_sub = rospy.Subscriber('pub_status', Int32, self.status_callback)
        time_sub = rospy.Subscriber('DataToBig', dataToAgent, self.time_callback)
        # Widgets
        self.title = tk.Label(
            self,
            bg='#000000',
            fg='#FFFFFF',
            text='DIT Robotics\nEurobot 2021',
            height=3,
            font=(default_font, 24)
        )
        self.title.pack(side=TOP)
        

        self.menu = tk.Listbox(
            self,
            font=(default_font, 20),
            bg='#000000',
            fg='#FFFFFF'
        )
        for item in strategy_vars:
            self.menu.insert(END, item)

        self.menu.pack(side=BOTTOM, pady=16)

        self.btn_confirm = tk.Button(
            self,
            text="Confirm",
            font=(default_font, 24),
            height=4,
            width=24,
            bg='#DB7093',
            activebackground='#FFC0CB',
            fg='#FFFFFF',
            command=self.push_confirm
        )
        self.btn_confirm.pack(side=BOTTOM, before=self.menu)

        self.btn_RST = tk.Button(
            self,
            text='Reset',
            font=(default_font, 24),
            height=4,
            width=24,
            bg='#DB7093',
            activebackground='#FFC0CB',
            fg='#FFFFFF',
            command=self.hit_wall
        )
        

        self.btn_PP = tk.Button(
            self,
            text='Prepare',
            font=(default_font, 24),
            height=4,
            width=24,
            bg='#DB7093',
            activebackground='#FFC0CB',
            fg='#FFFFFF',
            command = self.prepare
        )
        
        
        self.var = tk.StringVar()
        self.var.set('Push button plz ==')

        self.message = tk.Label(
            self,
            bg='#000000',
            fg='#FFFFFF',
            font=(default_font, 20),
            textvariable=self.var
        )
        
        
        self.timer = tk.Label(
            self,
            font=(default_font, 20),
            text="Game time: 0.00",
            bg='#000000',
            fg='#FFFFFF'
        )
        

        self.score_board = tk.Label(
            self,
            font=(default_font, 32),
            text="Score: %d" % self.current_score,
            bg='#000000',
            fg='#FFFFFF'
        )
        
    # Functions
    def push_confirm(self):
        if self.current_status == 0:
            mb_ans = mb.askquestion('Message', 'Are you sure that the strategy is correct?')
        else:
            mb_ans = mb.showerror('Error', 'Current state code is not zero! It is %d' % self.current_status)
        if mb_ans == 'yes':
            self.menu.forget()
            self.btn_confirm.forget()
            for i in range(len(strategy_vars)):
                if self.menu.select_includes(i):
                    strategy_pub.publish(i+1)
                    self.var.set('Reset completed\nCurrent script: %d' % (i+1))
            # This is demo for Porf. Chen:
            # status_pub.publish(5)
            # self.btn_PP.pack(side=BOTTOM)
            # self.message.pack(side=BOTTOM, before=self.btn_PP, pady=32)
            # self.timer.pack(side=BOTTOM, before=self.message)
            # while (self.current_status != 5):
            #     status_pub.publish(5)
            #     self.error_count += 1
            #     rospy.sleep(0.1)
            #     if (self.error_count >= 2000):
            #         self.error_count = 0
            #         mb.showerror('Error', 'Current state is not 5!')
            #         break
                
            # This is the real one:
            status_pub.publish(1)
            self.error_detect(1)
            self.btn_RST.pack(side=TOP)
            self.message.pack(side=BOTTOM, before=self.btn_RST, pady=32)
            self.timer.pack(side=BOTTOM, before=self.message)
            self.score_board.pack(side=BOTTOM, before=self.timer, pady=32)
            

            
    def score_callback(self, data):
        self.current_score = data.data
    
    def status_callback(self, data):
        self.current_status = data.data
    
    def time_callback(self, data):
        self.current_time = data.time


    def hit_wall(self):
        status_pub.publish(2)
        self.btn_RST.pack_forget()
        self.btn_PP.pack(side=TOP, padx=10)
        # self.var.set('Hit-wall positioning has started <3')
        # rospy.sleep(2.0)
        self.error_detect(2)
        status_pub.publish(3)
        self.error_detect(3)
        
                
    def prepare(self):
        # command_pub.publish(2000)
        # This is demo for Prof. Chen:
        # status_pub.publish(5)
        # This is the real one:
        status_pub.publish(4)
        self.error_detect(4)
        self.var.set('Go win! We ain\'t here to lose!')
        self.update_score()
        self.update_clock()
        

    def update_clock(self):
        if (self.current_status >= 4):
            self.timer.configure(text="Game time: %.2f" % self.current_time)
            self.master.after(10, self.update_clock)
    

    def update_score(self):
        if (self.current_status >= 4):
            self.score_board.configure(text='Score: %d' % self.current_score)
            self.master.after(100, self.update_score)
    
    def error_detect(self, state):
        if (state == 4):
            while (self.current_status != 4 and self.current_status != 5):
                status_pub.publish(state)
                self.error_count += 1
                if (self.error_count > 2000):
                    self.error_count = 0
                    mb.showerror('Error', 'Current state is not 4 or 5!\nPlease restart the launch file.')
        elif (state < 4):
            while (self.current_status != state):
                status_pub.publish(state)
                self.error_count += 1
                if (self.error_count > 2000):
                    self.error_count = 0
                    mb.showerror('Error', 'Current state is not %d!\nPlease restart the launch file.' % state)
        else:
            return
    
        
# END OF CLASS

if __name__ == '__main__':
    try:
        talker()
        status_pub.publish(0)
        root = tk.Tk()
        root.geometry('800x480')
        root.configure(bg='#000000')
        root.title('DIT Robotics Eurobot 2020')
        app = Application(root)
        app.configure(bg='#000000')
        root.mainloop()
    except rospy.ROSInterruptException:
        pass

# COLORS:
#FFB6C1 LightPink 浅粉红
#FFC0CB Pink 粉红
#DC143C Crimson 深红/猩红
#FFF0F5 LavenderBlush 淡紫红
#DB7093 PaleVioletRed 弱紫罗兰红
#FF69B4 HotPink 热情的粉红
#FF1493 DeepPink 深粉红
#C71585 MediumVioletRed 中紫罗兰红
#DA70D6 Orchid 暗紫色/兰花紫
#D8BFD8 Thistle 蓟色
#DDA0DD Plum 洋李色/李子紫
#EE82EE Violet 紫罗兰
#FF00FF Magenta 洋红/玫瑰红
#FF00FF Fuchsia 紫红/灯笼海棠
#8B008B DarkMagenta 深洋红
#800080 Purple 紫色
#BA55D3 MediumOrchid 中兰花紫
#9400D3 DarkViolet 暗紫罗兰
#9932CC DarkOrchid 暗兰花紫
#4B0082 Indigo 靛青/紫兰色
#8A2BE2 BlueViolet 蓝紫罗兰
#9370DB MediumPurple 中紫色
#7B68EE MediumSlateBlue 中暗蓝色/中板岩蓝
#6A5ACD SlateBlue 石蓝色/板岩蓝
#483D8B DarkSlateBlue 暗灰蓝色/暗板岩蓝
#E6E6FA Lavender 淡紫色/熏衣草淡紫
#F8F8FF GhostWhite 幽灵白
#0000FF Blue 纯蓝
#0000CD MediumBlue 中蓝色
#191970 MidnightBlue 午夜蓝
#00008B DarkBlue 暗蓝色
#000080 Navy 海军蓝
#4169E1 RoyalBlue 皇家蓝/宝蓝
#6495ED CornflowerBlue 矢车菊蓝
#B0C4DE LightSteelBlue 亮钢蓝
#778899 LightSlateGray 亮蓝灰/亮石板灰
#708090 SlateGray 灰石色/石板灰
#1E90FF DodgerBlue 闪兰色/道奇蓝
#F0F8FF AliceBlue 爱丽丝蓝
#4682B4 SteelBlue 钢蓝/铁青
#87CEFA LightSkyBlue 亮天蓝色
#87CEEB SkyBlue 天蓝色
#00BFFF DeepSkyBlue 深天蓝
#ADD8E6 LightBlue 亮蓝
#B0E0E6 PowderBlue 粉蓝色/火药青
#5F9EA0 CadetBlue 军兰色/军服蓝
#F0FFFF Azure 蔚蓝色
#E0FFFF LightCyan 淡青色
#AFEEEE PaleTurquoise 弱绿宝石
#00FFFF Cyan 青色
#00FFFF Aqua 浅绿色/水色
#00CED1 DarkTurquoise 暗绿宝石
#2F4F4F DarkSlateGray 暗瓦灰色/暗石板灰
#008B8B DarkCyan 暗青色
#008080 Teal 水鸭色
#48D1CC MediumTurquoise 中绿宝石
#20B2AA LightSeaGreen 浅海洋绿
#40E0D0 Turquoise 绿宝石
#7FFFD4 Aquamarine 宝石碧绿
#66CDAA MediumAquamarine 中宝石碧绿
#00FA9A MediumSpringGreen 中春绿色
#F5FFFA MintCream 薄荷奶油
#00FF7F SpringGreen 春绿色
#3CB371 MediumSeaGreen 中海洋绿
#2E8B57 SeaGreen 海洋绿
#F0FFF0 Honeydew 蜜色/蜜瓜色
#90EE90 LightGreen 淡绿色
#98FB98 PaleGreen 弱绿色
#8FBC8F DarkSeaGreen 暗海洋绿
#32CD32 LimeGreen 闪光深绿
#00FF00 Lime 闪光绿
#228B22 ForestGreen 森林绿
#008000 Green 纯绿
#006400 DarkGreen 暗绿色
#7FFF00 Chartreuse 黄绿色/查特酒绿
#7CFC00 LawnGreen 草绿色/草坪绿
#ADFF2F GreenYellow 绿黄色
#556B2F DarkOliveGreen 暗橄榄绿
#9ACD32 YellowGreen 黄绿色
#6B8E23 OliveDrab 橄榄褐色
#F5F5DC Beige 米色/灰棕色
#FAFAD2 LightGoldenrodYellow 亮菊黄
#FFFFF0 Ivory 象牙色
#FFFFE0 LightYellow 浅黄色
#FFFF00 Yellow 纯黄
#808000 Olive 橄榄
#BDB76B DarkKhaki 暗黄褐色/深卡叽布
#FFFACD LemonChiffon 柠檬绸
#EEE8AA PaleGoldenrod 灰菊黄/苍麒麟色
#F0E68C Khaki 黄褐色/卡叽布
#FFD700 Gold 金色
#FFF8DC Cornsilk 玉米丝色
#DAA520 Goldenrod 金菊黄
#B8860B DarkGoldenrod 暗金菊黄
#FFFAF0 FloralWhite 花的白色
#FDF5E6 OldLace 老花色/旧蕾丝
#F5DEB3 Wheat 浅黄色/小麦色
#FFE4B5 Moccasin 鹿皮色/鹿皮靴
#FFA500 Orange 橙色
#FFEFD5 PapayaWhip 番木色/番木瓜
#FFEBCD BlanchedAlmond 白杏色
#FFDEAD NavajoWhite 纳瓦白/土著白
#FAEBD7 AntiqueWhite 古董白
#D2B48C Tan 茶色
#DEB887 BurlyWood 硬木色
#FFE4C4 Bisque 陶坯黄
#FF8C00 DarkOrange 深橙色
#FAF0E6 Linen 亚麻布
#CD853F Peru 秘鲁色
#FFDAB9 PeachPuff 桃肉色
#F4A460 SandyBrown 沙棕色
#D2691E Chocolate 巧克力色
#8B4513 SaddleBrown 重褐色/马鞍棕色
#FFF5EE Seashell 海贝壳
#A0522D Sienna 黄土赭色
#FFA07A LightSalmon 浅鲑鱼肉色
#FF7F50 Coral 珊瑚
#FF4500 OrangeRed 橙红色
#E9967A DarkSalmon 深鲜肉/鲑鱼色
#FF6347 Tomato 番茄红
#FFE4E1 MistyRose 浅玫瑰色/薄雾玫瑰
#FA8072 Salmon 鲜肉/鲑鱼色
#FFFAFA Snow 雪白色
#F08080 LightCoral 淡珊瑚色
#BC8F8F RosyBrown 玫瑰棕色
#CD5C5C IndianRed 印度红
#FF0000 Red 纯红
#A52A2A Brown 棕色
#B22222 FireBrick 火砖色/耐火砖
#8B0000 DarkRed 深红色
#800000 Maroon 栗色
#FFFFFF White 纯白
#F5F5F5 WhiteSmoke 白烟
#DCDCDC Gainsboro 淡灰色
#D3D3D3 LightGrey 浅灰色
#C0C0C0 Silver 银灰色
#A9A9A9 DarkGray 深灰色
#808080 Gray 灰色
#696969 DimGray 暗淡灰
#000000 Black 纯黑
