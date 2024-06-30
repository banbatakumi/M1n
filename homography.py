import numpy

def find_homography(pixel_before_converting, pixel_after_converting):
    x1, y1 = pixel_before_converting[0]
    x2, y2 = pixel_before_converting[1]
    x3, y3 = pixel_before_converting[2]
    x4, y4 = pixel_before_converting[3]
    
    u1, v1 = pixel_after_converting[0]
    u2, v2 = pixel_after_converting[1]
    u3, v3 = pixel_after_converting[2]
    u4, v4 = pixel_after_converting[3]
    
    A = numpy.matrix([[ x1, y1, 1, 0,  0,  0, -x1*u1, -y1*u1, 0 ],
                      [ 0,  0,  0, x1, y1, 1, -x1*v1, -y1*v1, 0 ],
                      [ x2, y2, 1, 0,  0,  0, -x2*u2, -y2*u2, 0 ],
                      [ 0,  0,  0, x2, y2, 1, -x2*v2, -y2*v2, 0 ],
                      [ x3, y3, 1, 0,  0,  0, -x3*u3, -y3*u3, 0 ],
                      [ 0,  0,  0, x3, y3, 1, -x3*v3, -y3*v3, 0 ],
                      [ x4, y4, 1, 0,  0,  0, -x4*u4, -y4*u4, 0 ],
                      [ 0,  0,  0, x4, y4, 1, -x4*v4, -y4*v4, 0 ],
                      [ 0,  0,  0, 0,  0,  0, 0,      0,      1 ]]) 
    B = numpy.matrix([[ u1 ],
                      [ v1 ],
                      [ u2 ],
                      [ v2 ],
                      [ u3 ],
                      [ v3 ],
                      [ u4 ],
                      [ v4 ],
                      [ 1  ]])
    
    X = A.I * B
    X.shape = (3, 3)
    return X.tolist()


if __name__ == '__main__':
    pixel_before_converting = [[ -160, 0   ],
                               [ -160, 170 ],
                               [ 160,  0   ],
                               [ 160,  170 ]]
    pixel_after_converting = [[ -10,  0   ],
                              [ -160, 250 ],
                              [ 10,   0   ],
                              [ 160,  250 ]]
    
    X = find_homography(pixel_before_converting, pixel_after_converting)
    
    print("homegraphy_matrix: ")
    print(numpy.matrix(X))
