from time import time
import cv2
import numpy as np
from PIL import Image
from matplotlib import pyplot as plt
import yaml 
import cv2 as cv
import time
from scipy.interpolate import interp1d, splrep
import numpy as np
import scipy.io

def parse_ros_map(map_yaml, free_uncertain=False, interior_idx=None, exterior_idx=None):
    try:
        with open(map_yaml, "r") as stream:
            try:
                map_cfgs = yaml.safe_load(stream)
            except yaml.YAMLError as exc:
                print("Error reading file " + map_yaml + "!")
                print(exc)
                return 
    except:
        print("map_yaml is either not valid or is not a string!")
        return
        
    # Read in image
    im = np.array(Image.open(map_cfgs['image']))

    # Generate Coordinates mesh
    x,y = np.meshgrid(np.arange(im.shape[1]), np.arange(im.shape[0]))

    # Convert to float for scaling
    X,Y = x.astype(np.float64), y.astype(np.float64)

    # Takes into acount that the bottom left pixel is origin
    Y  = Y[::-1,:]

    # Scale by Resolution
    X *= map_cfgs['resolution']
    Y *= map_cfgs['resolution']

    # Shift to origin
    X  = X + map_cfgs['origin'][0]
    Y  = Y + map_cfgs['origin'][1]

    # if free_uncertain is true  => uncertain map points are occupied (black)
    uncertain_value = free_uncertain * 254

    free_thresh = map_cfgs['free_thresh']     * 255
    occu_thresh = map_cfgs['occupied_thresh'] * 255
    if(map_cfgs['mode'] == 'trinary'):
        im.setflags(write=1)
        im[np.logical_and(im > free_thresh, im < occu_thresh)] = uncertain_value
    elif(map_cfgs['mode'] == 'scale'):
        # As before if p > occupied_thresh, output the value 100. 
        # If p < free_thresh, output the value 0.
        # Otherwise, output 99 * (p - free_thresh) / (occupied_thresh - free_thresh)
        print("Mode not yet supported!")
        return
    else:
        print("Unsupported map mode : " + str(map_cfgs['mode']))

    free = im <= free_thresh
    occu = im >= occu_thresh
    im[free] = 0
    im[occu] = 254

    # Clean up the noise using a bilateral filter
    im =  cv.bilateralFilter(im,9,75,75)

    # Threshold to binary image
    im = np.where(im < free_thresh, 0, 1)

    # Flip image for proper morphological contour
    im_flipped = np.where(im == 0, 1, 0).astype('uint8')

    # Find first contour estimate by morphological gradient
    kernel = np.ones((5,5),np.uint8)
    im_flipped = cv2.morphologyEx(im_flipped.astype('uint8'), cv2.MORPH_GRADIENT, kernel, borderValue=1)

    # Edge detection generates contours
    im = cv2.Canny(im_flipped, 0,1)
    
    # https://pyimagesearch.com/2021/02/22/opencv-connected-component-labeling-and-analysis/
    # apply connected component analysis to the thresholded image 
    (numLabels, labels, stats, centroids) = cv2.connectedComponentsWithStats(im, 8, cv2.CV_32S)

    interior_mask = None
    exterior_mask = None
    # loop over the number of unique connected component labels 
    for i in range(0, numLabels):
        # if this is the first component then we examine the
        # *background* (typically we would just ignore this
        # component in our loop)
        if i == 0:
            text = "examining component {}/{} (background)".format(
                i + 1, numLabels)
        # otherwise, we are examining an actual connected component
        else:
            text = "examining component {}/{}".format( i + 1, numLabels)
        # print a status message update for the current connected
        # component
        print("[INFO] {}".format(text))

        # OpenCV Connected Component Labeling and Analysis
        # construct a mask for the current connected component by
        # finding a pixels in the labels array that have the current
        # connected component ID
        componentMask = (labels == i).astype("uint8") * 255
        
        # Component Masks as return
        if interior_idx is not None and exterior_idx is not None:
            if (i+1) == interior_idx:
                interior_mask = componentMask.copy()
            elif (i+1) == exterior_idx:
                exterior_mask = componentMask.copy()

        else:
            # show our output image and connected component mask
            cv2.imshow("Connected Component", componentMask)
            cv2.waitKey(0)

    # Flip back the image for proper visualization and interpretation
    im = np.where(im == 0, 1, 0).astype('uint8')

    return X, Y, im, interior_mask, exterior_mask

if __name__ == "__main__":
    map_yaml = "map1.yaml"
    interior_idx = 4
    exterior_idx = 3

    X,Y, im, interior_mask, exterior_mask = parse_ros_map(map_yaml, free_uncertain=True, interior_idx=interior_idx, exterior_idx=exterior_idx)

    # Generate Coordinates mesh
    x,y = np.meshgrid(np.arange(im.shape[0]), np.arange(im.shape[1]))

    if interior_mask is not None and exterior_mask is not None:
        interior = np.where(interior_mask == 0, 1, 0)
        exterior = np.where(exterior_mask == 0, 1, 0)
        
        # Sparse plotting
        X_interior, Y_interior = np.where(interior == 0)
        X_exterior, Y_exterior = np.where(exterior == 0)

        # Convert to map coordinates
        x_interior, y_interior = X[X_interior, Y_interior], Y[X_interior, Y_interior]
        x_exterior, y_exterior = X[X_exterior, Y_exterior], Y[X_exterior, Y_exterior]

        # Combine interior into a set of points
        interior_points = np.vstack((x_interior, y_interior)).T

        # Centerline generation
        center_points = []
        for x_ext, y_ext in zip(x_exterior, y_exterior):
            point = np.array([x_ext, y_ext])

            # find closest point idx in interior points
            closest_idx = np.argmin(np.linalg.norm(interior_points - point, axis=1))

            # get closest point
            closest_point = interior_points[closest_idx]

            # find midpoint
            midpoint = (point + closest_point)/2
            
            center_points.append(midpoint)

        center_points = np.array(center_points)
                
        # Plot Separately (keep desired)
        # plt.scatter(x_interior, y_interior, c=im[X_interior, Y_interior], cmap='gray')
        # plt.scatter(x_exterior, y_exterior, c=im[X_exterior, Y_exterior], cmap='gray')
        plt.scatter(x_interior, y_interior, c='g')
        plt.scatter(x_exterior, y_exterior, c='b')
        plt.scatter(*center_points.T, c='black')
        plt.show()


    # # Define some points:
    # data = np.genfromtxt('wp-2022-03-28-01-32-41.csv', delimiter=',')
    # center = data[:,0:2]
    # center = np.vstack((center, center[0]))

    # # Linear length along the line:
    # distance = np.cumsum( np.sqrt(np.sum( np.diff(center, axis=0)**2, axis=1 )) )
    # distance = np.insert(distance, 0, 0)/distance[-1]

    # # Interpolation for different methods:
    # kind = 'linear'

    # interpolations_methods = [kind]

    # resolution = 1805
    # interpolations_per_segment = int(np.floor((2*resolution)/len(center)))
    # alpha = np.linspace(0, 1, resolution)

    # interpolator =  interp1d(distance, center, kind=kind, axis=0)
    # interpolated_center = interpolator(alpha)

    # interpolated_center = {}
    # for method in interpolations_methods:
    #     interpolator =  interp1d(distance, center, kind='quadratic', axis=0)
    #     interpolated_center[method] = interpolator(alpha)
    #     # interpolated_center[method] = np.delete(interpolated_center[method], [0,1,2,3,4,5], 0)

    # # Remove intitial segment of interpolated center
    # interpolated_center[kind] = interpolated_center[kind][:resolution,:] 

    # # Define some points:
    # data = np.genfromtxt('wp-2022-03-28-02-21-23.csv', delimiter=',')
    # interior = data[:,0:2]
    # interior = np.vstack((interior, interior[0]))

    # # Linear length along the line:
    # distance = np.cumsum( np.sqrt(np.sum( np.diff(interior, axis=0)**2, axis=1 )) )
    # distance = np.insert(distance, 0, 0)/distance[-1]

    # interpolated_interior = interpolator(alpha)

    # interpolated_interior = {}
    # for method in interpolations_methods:
    #     interpolator =  interp1d(distance, interior, kind='slinear', axis=0)
    #     interpolated_interior[method] = interpolator(alpha)
    #     # interpolated_interior[method] = np.delete(interpolated_interior[method], [0,1,2,3,4,5], 0)

    # # Remove intitial segment of interpolated interior
    # interpolated_interior[kind] = interpolated_interior[kind][:resolution,:] 

    # # Define some points:
    # data = np.genfromtxt('wp-2022-03-28-02-18-44.csv', delimiter=',')
    # exterior = data[:,0:2]
    # exterior = np.vstack((exterior, exterior[0]))

    # # Linear length along the line:
    # distance = np.cumsum( np.sqrt(np.sum( np.diff(exterior, axis=0)**2, axis=1 )) )
    # distance = np.insert(distance, 0, 0)/distance[-1]

    # interpolated_exterior = interpolator(alpha)

    # interpolated_exterior = {}
    # for method in interpolations_methods:
    #     interpolator =  interp1d(distance, exterior, kind='slinear', axis=0)
    #     interpolated_exterior[method] = interpolator(alpha)
    #     # interpolated_exterior[method] = np.delete(interpolated_exterior[method], [0,1,2,3,4,5], 0)

    # # Remove intitial segment of interpolated exterior
    # interpolated_exterior[kind] = interpolated_exterior[kind][:resolution,:] 

    # # Graph:
    # for method_name, curve in interpolated_center.items():
    #     plt.scatter(*curve.T, label=method_name + ' center')

    # plt.plot(*center.T, 'ok', label='original center')
    # plt.scatter(0,0, c='red')
    # idx = 0
    # plt.scatter(interpolated_center[kind][idx,0], interpolated_center[kind][idx,1], s=200, c='green')
    # plt.axis('equal'); plt.legend(); plt.xlabel('x'); plt.ylabel('y')

    # # Graph:
    # for method_name, curve in interpolated_interior.items():
    #     plt.scatter(*curve.T, label=method_name + ' interior')

    # plt.plot(*interior.T, 'ok', label='original interior')
    # plt.scatter(0,0, c='red')
    # idx = 0
    # plt.scatter(interpolated_interior[kind][idx,0], interpolated_interior[kind][idx,1], s=200, c='green')
    # plt.axis('equal'); plt.legend(); plt.xlabel('x'); plt.ylabel('y')

    # # Graph:
    # for method_name, curve in interpolated_exterior.items():
    #     plt.scatter(*curve.T, label=method_name + ' exterior')

    # plt.plot(*exterior.T, 'ok', label='original exterior')
    # plt.scatter(0,0, c='red')
    # idx = 0
    # plt.scatter(interpolated_exterior[kind][idx,0], interpolated_exterior[kind][idx,1], s=200, c='green')
    # plt.axis('equal'); plt.legend(); plt.xlabel('x'); plt.ylabel('y')
    
    # # new_interior = interpolated_exterior[kind].copy().T
    # # test = (new_interior - np.mean(new_interior,axis=1).reshape(-1,1))*0.5

    # # test += np.mean(new_interior,axis=1).reshape(-1,1)
    # # plt.scatter(*test)
    # plt.show()

    # inner  = interpolated_interior[kind][:].T
    # outer  = interpolated_exterior[kind][:].T
    # center =   interpolated_center[kind][:].T
    
    # scipy.io.savemat('MPCC/Matlab/Tracks/levineTrack.mat', mdict={'levineTrack': {'inner':inner, 'outer':outer, 'center':center}})
    

