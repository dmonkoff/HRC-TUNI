#!/usr/bin/env python
import sys
import cv2
import numpy as np
import yaml

class Pattern:
    def __init__(self, indentifier, texture, location=np.array([0, 0])):
        self._location = location  # left corner of the pattern y, x
        self._identifier = indentifier
        self._texture = cv2.imread(texture)
        self._orig_texture = cv2.imread(texture)

class Projector:
    def __init__(self, save_file=None):
        self.patterns = []
        self.current_pattern_idx = 0
        # configs
        self.screen = np.zeros([1080, 1920, 3], dtype=np.uint8)
        self.step_size = 5
        if save_file is not None:
            self.save_file = save_file

    def update_screen(self):
        # put pattern on the screen
        self.screen = np.zeros([self.screen.shape[0], self.screen.shape[1], 3], dtype=np.uint8)
        for i in range(len(self.patterns)):
            if i == self.current_pattern_idx:
                continue

            pat = self.patterns[i]
            sh, sw = pat._texture.shape[:2]
            self.screen[pat._location[0]:pat._location[0]+sh, pat._location[1]:pat._location[1] + sw] = pat._texture

        pat = self.patterns[self.current_pattern_idx]
        sh, sw = pat._texture.shape[:2]
        self.screen[pat._location[0]:pat._location[0] + sh, pat._location[1]:pat._location[1] + sw] = pat._texture

        cv2.circle(self.screen, (int(pat._location[1] + (sh / 2)), int(pat._location[0] + (sw / 2))), 15, (0, 0, 255), -1)

    def move_pattern(self, x, y):
        pat = self.patterns[self.current_pattern_idx]
        sh, sw = pat._texture.shape[:2]
        if pat._location[0]+y+sh > self.screen.shape[0] or pat._location[0]+y < 0:
            return

        if pat._location[1]+x+sw > self.screen.shape[1] or pat._location[1]+x < 0:
            return

        pat._location[0] = pat._location[0] + y
        pat._location[1] = pat._location[1] + x

    def read_components(self, components):
        for i in range(len(components)):
            # TODO: why we have to give location=np.array([0,  0]), for some reason then every component shares the location
            pattern = Pattern(components[i][0], components[i][1], location=np.array([0,  0]))
            self.patterns.append(pattern)
        self.current_pattern_idx = 0

    def read_from_configs(self, path):
        with open(path + 'projector_buttons.yaml', 'r') as f:
            data = yaml.safe_load(f)
        keys = data.keys()
        for i in range(len(keys)):
            pattern = Pattern(keys[i], path + data[keys[i]]['img_path'], np.array(data[keys[i]]['loc']))
            self.patterns.append(pattern)
        self.current_pattern_idx = 0
        # print(data)

    def rotate_pattern(self, k):
        from scipy.ndimage import rotate
        sh, sw = self.screen.shape[:2]
        pat = self.patterns[self.current_pattern_idx]

        # y, x = screen.shape
        if pat._location[0] + pat._texture.shape[1] > self.screen.shape[0]:
            return

        if pat._location[1] + pat._texture.shape[0] > self.screen.shape[1]:
            return

        pat._texture = rotate(pat._texture, k)
        ph, pw = pat._texture.shape[:2]

        if ph > sh or pw > sw:
            ratio = float(sh) / float(ph)
            print("Screen resolution smaller than the pattern, resizing")
            self.resize_pattern(pat, ratio, ratio)

    def switch_pattern(self):
        if self.current_pattern_idx + 1 >= len(self.patterns):
            self.current_pattern_idx = 0
        else:
            self.current_pattern_idx += 1

    def save_and_quit(self):
        full_save_path = self.save_file + "projector_buttons.yaml"
        dic = {}
        for i in range(len(self.patterns)):
            dic2 = {}
            dic2['loc'] = self.patterns[i]._location.tolist()
            dic2['img_path'] = self.patterns[i]._identifier + '.png'
            dic[self.patterns[i]._identifier] = dic2
            print("ID, IMG_PATH, LOC: %s, %s, %s" % (self.patterns[i]._identifier, dic2['img_path'], dic2['loc']))
            cv2.imwrite(self.save_file + '/' + self.patterns[i]._identifier + '.png', self.patterns[i]._texture)
        with open(full_save_path, 'w+') as f:
            yaml.dump(dic, f, default_flow_style=False, allow_unicode=True)
        print("Done!")

    def save_pattern(self):
        '''
        Saves the orientation and size
        :return:
        '''
        self.orig_patterns[self.current_pattern_idx] = self.pattern

    def plot_image(self):
        self.update_screen()
        cv2.namedWindow("window", cv2.WND_PROP_FULLSCREEN)
        cv2.setWindowProperty("window", cv2.WND_PROP_FULLSCREEN, cv2.WINDOW_FULLSCREEN)
        cv2.imshow("window", self.screen)
        print("Screen shape: ", self.screen.shape)
        while 1:
            k = cv2.waitKey(33)
            # print(k)
            if k == 113:  # q
                cv2.destroyAllWindows()
                return
            elif k == -1 or k == 255:  # normally -1 returned,so don't print it
                continue
            elif k == 81:
                #print "left"  # else print its value
                self.move_pattern(-1*self.step_size, 0)
            elif k == 82:
                #print "up"
                self.move_pattern(0, -1*self.step_size)
            elif k == 83:
                #print "right"
                self.move_pattern(1*self.step_size, 0)
            elif k == 84:
                #print "down"
                self.move_pattern(0, 1*self.step_size)
            elif k == ord('s'):  # s
                self.resize_pattern(self.patterns[self.current_pattern_idx]._orig_texture,
                                    int(self.patterns[self.current_pattern_idx]._texture.shape[1] * 0.99),
                                    int(self.patterns[self.current_pattern_idx]._texture.shape[0] * 0.99), False)
            elif k == ord('b'):  # b
                self.resize_pattern(self.patterns[self.current_pattern_idx]._orig_texture,
                                    int(self.patterns[self.current_pattern_idx]._texture.shape[1] * 1.01),
                                    int(self.patterns[self.current_pattern_idx]._texture.shape[0] * 1.01), False)
            elif k == ord('r'):  # r
                self.rotate_pattern(90)
            elif k == ord('m'):  # m
                self.switch_pattern()
            elif k == ord('p'):  # p
                self.save_pattern()
            elif k == ord('l'):
                self.save_and_quit()
                return
            self.update_screen()
            cv2.imshow("window", self.screen)

    def resize_pattern(self, pattern, x, y, scaling=True):
        pat = self.patterns[self.current_pattern_idx]
        if scaling:
            pat._texture = cv2.resize(pattern, None, fx=x, fy=y, interpolation=cv2.INTER_AREA)
        else:
            pat._texture = cv2.resize(pattern, (x, y), interpolation=cv2.INTER_AREA)
        print("Pattern shape: ", pat._texture.shape)



def main(args):


    save_file = '/home/antti/work/catkin_ws/src/HRC-TUNI/unity_msgs/configs/mobile_demo/'
    cb = Projector(save_file)
    components = \
    [
        ['GO', '/home/antti/work/catkin_ws/src/unity/unity_msgs/configs/go.png'],
        ['STOP', '/home/antti/work/catkin_ws/src/unity/unity_msgs/configs/stop.png'],
        ['CONFIRM', '/home/antti/work/catkin_ws/src/unity/unity_msgs/configs/confirm_object.png'],
        ['DEAD_MAN', '/home/antti/work/catkin_ws/src/unity/unity_msgs/configs/dead_mans_witch.png'],
        ['ROBOT_ACTIVE', '/home/antti/work/catkin_ws/src/unity/unity_msgs/configs/robot_active.png'],
        ['START_ROBOT', '/home/antti/work/catkin_ws/src/unity/unity_msgs/configs/start_robot.png'],
        ['DM_NOT_PRESSED', '/home/antti/work/catkin_ws/src/unity/unity_msgs/configs/dead_man_not_pressed.png']
    ]
    # cb.read_components(components)
    cb.read_from_configs(save_file)
    cb.plot_image()


if __name__ == '__main__':
    main(sys.argv)