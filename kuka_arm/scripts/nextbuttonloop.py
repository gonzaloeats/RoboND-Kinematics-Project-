import pyautogui, time
# for i in range(10):
#     pyautogui.moveRel(100, 0, duration=0.25)
#     pyautogui.moveRel(0, 100, duration=0.25)
#     pyautogui.moveRel(-100, 0, duration=0.25)
#     pyautogui.moveRel(0, -100, duration=0.25)

# next button positoin is (285, 506)
n = 5000 
while n > 0:
    pyautogui.click(285,506)
    time.sleep(30)
    n -= 1
