import time
import numpy as np
from tkinter import *
import cv2
from threading import Thread, Lock
from PIL import Image, ImageTk

color = np.array([[0, 0, 0], [255, 255, 255]])  # 设置色块颜色阈值，下限，上限
flag = 1
capture_flag = 1
out = 0
color_img = []
image = []
photo = []
temp_photo = []


class GUI():
    def __init__(self):
        global photo, temp_photo
        self.root = Tk()

        self.root.title('菜单')
        self.root.geometry('1300x600')  # 这里的乘号不是 * ，而是小写英文字母 x

        back = Image.open('back.jpg')
        back = back.resize((1300, 600))
        back = ImageTk.PhotoImage(image=back)  # 修改这里，避免之前的错误
        self.back_label = Label(self.root, image=back)
        self.back_label.place(relx=0.0, rely=0.0, relwidth=1, relheight=1)

        self.var_0 = DoubleVar()
        self.var_1 = DoubleVar()
        self.var_2 = DoubleVar()
        self.var_3 = DoubleVar()
        self.var_4 = DoubleVar()
        self.var_5 = DoubleVar()

        self.paramVar = StringVar()
        # MIN_B
        self.scale_min_B = Scale(self.root, orient=HORIZONTAL, length=10, from_=0.0, to=255.0, label='MIN_色调',
                                 tickinterval=15,
                                 resolution=1, variable=self.var_0, repeatdelay=10)
        self.scale_min_B.place(relx=0.005, rely=0.0, relwidth=0.5, relheight=0.1)
        self.scale_min_B.set(color[0][0])
        self.scale_min_B.bind('<ButtonRelease-1>', self.scale_min_B_event)

        # MIN_G
        self.scale_min_G = Scale(self.root, orient=HORIZONTAL, length=1000, from_=0.0, to=255.0, label='MIN_色饱和度',
                                 tickinterval=15,
                                 resolution=1, variable=self.var_1, repeatdelay=10)
        self.scale_min_G.place(relx=0.005, rely=0.1, relwidth=0.5, relheight=0.1)
        self.scale_min_G.set(color[0][1])
        self.scale_min_G.bind('<ButtonRelease-1>', self.scale_min_G_event)

        # MIN_R
        self.scale_min_R = Scale(self.root, orient=HORIZONTAL, length=1000, from_=0.0, to=255.0, label='MIN_亮度',
                                 tickinterval=15,
                                 resolution=1, variable=self.var_2, repeatdelay=10)
        self.scale_min_R.place(relx=0.005, rely=0.2, relwidth=0.5, relheight=0.1)
        self.scale_min_R.set(color[0][2])
        self.scale_min_R.bind('<ButtonRelease-1>', self.scale_min_R_event)

        # MAX_B
        self.scale_max_B = Scale(self.root, orient=HORIZONTAL, length=1000, from_=0.0, to=255.0, label='MAX_色调',
                                 tickinterval=15,
                                 resolution=1, variable=self.var_3, repeatdelay=10)
        self.scale_max_B.place(relx=0.005, rely=0.3, relwidth=0.5, relheight=0.1)
        self.scale_max_B.set(color[1][0])
        self.scale_max_B.bind('<ButtonRelease-1>', self.scale_max_B_event)

        # MAX_G
        self.scale_max_G = Scale(self.root, orient=HORIZONTAL, length=1000, from_=0.0, to=255.0, label='MAX_色饱和度',
                                 tickinterval=15,
                                 resolution=1, variable=self.var_4, repeatdelay=10)
        self.scale_max_G.place(relx=0.005, rely=0.4, relwidth=0.5, relheight=0.1)
        self.scale_max_G.set(color[1][1])
        self.scale_max_G.bind('<ButtonRelease-1>', self.scale_max_G_event)

        # MAX_R
        self.scale_max_R = Scale(self.root, orient=HORIZONTAL, length=1000, from_=0.0, to=255.0, label='MAX_亮度',
                                 tickinterval=15,
                                 resolution=1, variable=self.var_5, repeatdelay=10)
        self.scale_max_R.place(relx=0.005, rely=0.5, relwidth=0.5, relheight=0.1)
        self.scale_max_R.set(color[1][2])
        self.scale_max_R.bind('<ButtonRelease-1>', self.scale_max_R_event)

        capture_button = Button(self.root, text='截取图片', command=self.capture)
        capture_button.place(relx=0.005, rely=0.65, relwidth=0.1, relheight=0.05)

        open_button = Button(self.root, text='恢复视频', command=self.open)
        open_button.place(relx=0.155, rely=0.65, relwidth=0.1, relheight=0.05)

        out_button = Button(self.root, text='输出结果', command=self.out_)
        out_button.place(relx=0.305, rely=0.65, relwidth=0.1, relheight=0.05)

        enter_button = Button(self.root, text='确认', command=self.enter)
        enter_button.place(relx=0.225, rely=0.775, relwidth=0.1, relheight=0.05)

        self.entry_ = Entry(self.root, width=50)
        self.entry_.place(relx=0.015, rely=0.75, relwidth=0.2, relheight=0.1)

        image = Image.open('init.png')
        image = image.resize((640, 320))
        photo = ImageTk.PhotoImage(image=image)  # 修改这里，避免之前的错误
        temp_photo = photo
        self.image_label = Label(self.root, image=photo)
        self.image_label.place(relx=0.55, rely=0.005, relwidth=0.44, relheight=0.45)

        image2 = Image.open('use.png')
        image2 = image2.resize((640, 320))
        image2 = ImageTk.PhotoImage(image=image2)  # 修改这里，避免之前的错误
        self.image2_label = Label(self.root, image=image2)
        self.image2_label.place(relx=0.55, rely=0.5, relwidth=0.44, relheight=0.45)

        background_thread = Thread(target=self.update_image)  # 创建线程  用于更新图片
        background_thread.start()

        self.root.mainloop()  # 显示窗口，这个代码一定要放在所有窗口设置的后面

    def scale_min_B_event(self, event):
        color[0][0] = int(self.scale_min_B.get())

    def scale_min_G_event(self, event):
        color[0][1] = int(self.scale_min_G.get())

    def scale_min_R_event(self, event):
        color[0][2] = int(self.scale_min_R.get())

    def scale_max_B_event(self, event):
        color[1][0] = int(self.scale_max_B.get())

    def scale_max_G_event(self, event):
        color[1][1] = int(self.scale_max_G.get())

    def scale_max_R_event(self, event):
        color[1][2] = int(self.scale_max_R.get())

    def capture(self):
        global capture_flag
        capture_flag = 0

    def open(self):
        global capture_flag
        capture_flag = 1

    def out_(self):
        global out
        out = 1

    def enter(self):
        global color
        datas = self.entry_.get()
        datas += ' '
        temp = datas[0]
        i = 0
        j = 0
        for data in datas[1:]:
            if data != ' ' and data != ',' and data != '，':
                temp += data

            else:
                color[j][i] = int(temp)
                i += 1
                if i == 3:
                    j += 1
                    i = 0
                temp = ''
        print(color)

    def update_image(self):
        global photo, temp_photo
        photo = temp_photo
        self.image_label.config(image=photo)
        self.root.after(30, self.update_image)

        # if(len(data))


def concat_1(img_1, img_2, img_shape):
    img_1 = cv2.resize(img_1, img_shape)
    img_2 = cv2.resize(img_2, img_shape)
    img_o = cv2.hconcat([img_1, img_2])
    return img_o


def GUI_task():
    global flag
    GUI()
    flag = 0


def out_color(color):
    print("color == [[", color[0][0], ",", color[0][1], ",", color[0][2], "] [", color[1][0], ",", color[1][1], ",",
          color[1][2], "]]")


def CV_task(kinds, source):
    global color, flag, capture_flag, color_img, image, out, photo, temp_photo
    use_img = cv2.imread('./use.png')
    if kinds == 2:
        cap = cv2.VideoCapture(source)
        if cap.isOpened():
            pass
        else:
            print("error to open capture")
        while flag:
            ret, frame = cap.read()
            if frame is None:
                break
            if ret:
                if capture_flag:
                    image = cv2.resize(frame, (640, 640))
                u_image = image.copy()
                u_image = cv2.cvtColor(u_image, cv2.COLOR_BGR2HSV)
                u_image = cv2.GaussianBlur(u_image, (5, 5), 0)  # 高斯滤波降噪，模糊图片
                color_img = cv2.inRange(u_image, color[0], color[1])
                color_img = cv2.cvtColor(color_img, cv2.COLOR_GRAY2RGB)
                concat_img = concat_1(image, color_img, (320, 320))
                # cv2.imshow('white is true ',concat_img)

                image_cv = cv2.cvtColor(concat_img, cv2.COLOR_BGR2RGB)
                image_pil = Image.fromarray(image_cv)
                temp_photo = ImageTk.PhotoImage(image_pil)

                keyCode = cv2.waitKey(30) & 0xFF
                if keyCode == 27:  # ESC键退出
                    break
                if out:
                    out = 0
                    out_color(color)

        cap.release()
    elif kinds == 1:
        frame = cv2.imread(source)
        while flag:
            image = cv2.resize(frame, (640, 640))
            u_image = image.copy()
            u_image = cv2.cvtColor(u_image, cv2.COLOR_BGR2HSV)
            u_image = cv2.GaussianBlur(u_image, (5, 5), 0)  # 高斯滤波降噪，模糊图片
            color_img = cv2.inRange(u_image, color[0], color[1])
            color_img = cv2.cvtColor(color_img, cv2.COLOR_GRAY2RGB)
            concat_img = concat_1(image, color_img, (320, 320))
            # cv2.imshow('white is true ', concat_img)

            image_cv = cv2.cvtColor(concat_img, cv2.COLOR_BGR2RGB)
            image_pil = Image.fromarray(image_cv)
            temp_photo = ImageTk.PhotoImage(image_pil)

            keyCode = cv2.waitKey(30) & 0xFF
            if out:
                out = 0
                out_color(color)
            if keyCode == 27:  # ESC键退出
                break

    cv2.destroyAllWindows()
    out_color(color)


if __name__ == "__main__":
    Gui_task = Thread(target=GUI_task, args="", name="GUI_Task")
    kinds = 2  # 选择用视频还是图片   1表示用图片  2 表示用视频
    source = 0  # "./test.png" #如果是图片，则是选择路径，如果是视频，则是选择摄像头，如填入0选择摄像头0 注：0不用打单引号
    Cv_task = Thread(target=CV_task, args=(kinds, source), name="CV_Task")
    Gui_task.start()
    Cv_task.start()
