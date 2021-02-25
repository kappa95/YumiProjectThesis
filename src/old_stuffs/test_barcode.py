import os
import zbar
import cv2

filename = os.path.expanduser('~/image.bmp')
# filename = 'provetta3.jpg'
cv2.namedWindow('output', cv2.WINDOW_NORMAL)        # Create window with freedom of dimensions
im = cv2.imread(filename, cv2.IMREAD_GRAYSCALE)
imS = cv2.resize(im, (960, 540))
cv2.imshow('output', im)
cv2.waitKey(0)
cv2.destroyAllWindows()
scanner = zbar.Scanner()
results = scanner.scan(im)
if not results:
    print('oggetto nullo')
else:
    print(results.pop().data)
    # for result in results:
    #     print(result)

