
import sys
import os
import math
import array
import json

   
try:
    with open('/sys/firmware/devicetree/base/model') as model:     
        RPi_model = model.read()
    if RPi_model[:12]== "Raspberry Pi":
        pass                         # will be running on Raspberry Pi
    else:
        # will be running on desktop computer
        import wx
        import random
except Exception:
    # will be running on desktop computer
    import wx
    import random                # 0 - Simulation without physics, 
                                        # 1 - Simulation synchronous with physics, 
                                        # 3 - Simulation streaming with physics



uprightRobotRadius = 0.2  # Radius to walk around an upright robot (in m).


class Glob:
    def __init__(self):
        self.COLUMNS = 18
        self.ROWS = 13
        self.pf_coord =   [0.276, 0.749, 2]  #[-0.4, 0.0 , 0] # [0.5, 0.5 , -math.pi * 3/4]
        self.ball_coord = [-0.132, 0.957]        #[0, 0]
        self.obstacles = [[0.4, 0.025, 0.2], [0.725, -0.475, 0.2]]  #[[0, 0, 0.15], [0.4, 0.025, 0.2], [0.725, -0.475, 0.2]]
        #self.ball_coord = [2, 0]
        #self.obstacles = [[0.4, 0.025, 0.2], [0.725, -0.475, 0.2], [0.8, 0.55, 0.2], [1.175, 0, 0.2], [1.625, -0.4, 0.2], [1.7, 0.425, 0.2]]
        self.landmarks = {"post1": [[ 1.8, -0.6 ]], "post2": [[ 1.8, 0.6 ]], "post3": [[ -1.8, 0.6 ]], "post4": [[ -1.8, -0.6 ]],
                          "unsorted_posts": [[ 1.8, 0.6 ],[ 1.8, -0.6 ],[ -1.8, 0.6 ],[ -1.8, -0.6 ]],
                          "FIELD_WIDTH": 2.6, "FIELD_LENGTH": 3.6 }
        self.params = {'CYCLE_STEP_YIELD': 103.5}
        self.cycle_step_yield = 103.5
        current_work_directory = os.getcwd()
        current_work_directory = current_work_directory.replace('\\', '/')
        current_work_directory += '/'
        self.strategy_data = array.array('b',(0 for i in range(self.COLUMNS * self.ROWS * 2)))
        self.import_strategy_data(current_work_directory)

    def import_strategy_data(self, current_work_directory):
        with open(current_work_directory + "Init_params/strategy_data.json", "r") as f:
            loaded_Dict = json.loads(f.read())
        if loaded_Dict.get('strategy_data') != None:
            strategy_data = loaded_Dict['strategy_data']
        for column in range(self.COLUMNS):
            for row in range(self.ROWS):
                index1 = column * self.ROWS + row
                power = strategy_data[index1][2]
                yaw = int(strategy_data[index1][3] * 40)  # yaw in radians multiplied by 40
                self.strategy_data[index1*2] = power
                self.strategy_data[index1*2+1] = yaw

class Soccer_monitor(wx.Frame):

    def __init__(self, *args, **kw):
        #super().__init__(*args, **kw)
        super(Soccer_monitor, self).__init__(*args, **kw)
        self.glob = Glob()
        self.isLeftDown = False
        self.InitUI()

    def InitUI(self):

        self.Bind(wx.EVT_PAINT, self.OnPaint)
        self.Bind(wx.EVT_LEFT_DOWN, self.OnLeftDown)
        self.Bind(wx.EVT_LEFT_UP, self.OnLeftUp)
        self.Bind(wx.EVT_MOTION, self.OnMove)

        self.SetTitle('Lines')
        self.Centre()

    def OnPaint(self, e):
        self.dc = wx.PaintDC(self)
        #dc.SetBackground(wx.Brush('#1ac500'))
        #dc.SetPen(wx.Pen('#d4d4d4'))
        self.SetClientSize(800, 600)
        self.dc.SetBrush(wx.Brush('#1ac500'))
        self.dc.DrawRectangle(0, 0, 800, 600)
        pen = wx.Pen('#ffffff', 10, wx.SOLID)
        pen.SetJoin(wx.JOIN_MITER)
        self.dc.SetPen(pen)
        self.dc.DrawRectangle(40, 40, 720, 520)
        self.dc.DrawRectangle(0, 200, 40, 200)
        self.dc.DrawRectangle(760, 200, 40, 200)
        self.dc.DrawRectangle(40, 160, 40, 280)
        self.dc.DrawRectangle(720, 160, 40, 280)
        self.dc.DrawCircle(400,300,60)
        self.dc.DrawLine(400,45,400,555)
        self.dc.DrawLine(390,300,410,300)
        self.dc.DrawLine(210,300,230,300)
        self.dc.DrawLine(220,290,220,310)
        self.dc.DrawLine(570,300,590,300)
        self.dc.DrawLine(580,290,580,310)
        
        self.dc.SetAxisOrientation(True, True)
        self.dc.SetDeviceOrigin(400, 300)

        pen1 = wx.Pen('#000000', 1, wx.SOLID)
        self.dc.SetPen(pen1)
        self.dc.SetBrush(wx.Brush('#ffffff'))
        self.dc.DrawCircle(int(self.glob.pf_coord[0] *200), int(self.glob.pf_coord[1] *200) , int(0.1 *200))  #robot
        self.dc.SetBrush(wx.Brush('#000000'))
        for obstacle in self.glob.obstacles:
            self.dc.DrawCircle(int(obstacle[0] *200), int(obstacle[1] *200), int(obstacle[2] *100))      # obstacle
        self.dc.SetBrush(wx.Brush('#ff0000'))
        self.dc.DrawCircle(int(self.glob.ball_coord[0] *200), int(self.glob.ball_coord[1] *200), int(0.04 *200))   # ball
        

        #dc.Bind()

    def draw_arc(self, x1, y1, x2, y2, cx, cy, CW):
        self.dc.SetBrush(wx.Brush('#ff0000', wx.BRUSHSTYLE_TRANSPARENT))
        if CW: self.dc.DrawArc(x2, y2, x1, y1, cx, cy)
        else: self.dc.DrawArc(x1, y1, x2, y2, cx, cy)

    def OnLeftDown(self, event):
        #dc = wx.ClientDC(self.staticBMP)
        pos = event.GetLogicalPosition(self.dc)
        self.isLeftDown = True
        if math.sqrt((pos[0]-self.glob.pf_coord[0]*200)**2 + (pos[1]-self.glob.pf_coord[1]*200)**2) <= 20: 
            self.moving_object = -1       #'robot'
            self.dx = self.glob.pf_coord[0]*200 - pos[0]
            self.dy = self.glob.pf_coord[1]*200 - pos[1]
        elif math.sqrt((pos[0]-self.glob.ball_coord[0]*200)**2 + (pos[1]-self.glob.ball_coord[1]*200)**2) <= 8: 
            self.moving_object = -2    #'ball'
            self.dx = self.glob.ball_coord[0]*200 - pos[0]
            self.dy = self.glob.ball_coord[1]*200 - pos[1]
        else: self.isLeftDown = False
        if not self.isLeftDown:
            for obstacle in self.glob.obstacles:
                if math.sqrt((pos[0]-obstacle[0]*200)**2 + (pos[1]-obstacle[1]*200)**2) <= 20:
                    self.isLeftDown = True
                    self.moving_object = self.glob.obstacles.index(obstacle)      #'obstacle'
                    self.dx = obstacle[0]*200 - pos[0]
                    self.dy = obstacle[1]*200 - pos[1]
                    break
        #dc.DrawCircle(pos[0], pos[1], 5)
        a = 1
        
    def OnLeftUp(self, event):
        self.isLeftDown = False

    def OnMove(self, event):
        if self.isLeftDown:
            pos = event.GetLogicalPosition(self.dc)
            if self.moving_object == -1:                         #'robot'
                self.glob.pf_coord = [(pos[0] + self.dx)/200, (pos[1] + self.dy)/200, self.glob.pf_coord[2]]
                self.Refresh()
            if self.moving_object == -2:                          # 'ball'
                self.glob.ball_coord = [(pos[0] + self.dx)/200, (pos[1] + self.dy)/200]
                #self.glob.obstacles[0] = [self.glob.ball_coord[0], self.glob.ball_coord[1], 0.15]
                self.Refresh()
            if self.moving_object >= 0:                             # 'obstacle'
                self.glob.obstacles[self.moving_object] = [(pos[0] + self.dx)/200, (pos[1] + self.dy)/200, self.glob.obstacles[self.moving_object][2]]
                self.Refresh()

def launcher():

    app = wx.App()
    ex = Soccer_monitor(None)
    ex.Show()
    app.MainLoop()




if __name__ == '__main__':
    launcher()