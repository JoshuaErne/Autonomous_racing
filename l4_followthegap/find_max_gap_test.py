import numpy as np

def find_max_gap(free_space_ranges):
    """ Return the start index & end index of the max gap in free_space_ranges
    # """
    # THRESHOLD = 5.0

    # gap_width = 2
    # gap_start = 0
    
    # max_gap   = [0, 0]

    # found_gap = False
    # for i in range(2, len(free_space_ranges)):
    #     for j in range(gap_width + 1):
    #         if(free_space_ranges[gap_start + j] < THRESHOLD):
    #             gap_width = 2
    #             gap_start = j + 1
    #             i += j
    #             break

    #     if(gap_width + 1 > (max_gap[1] - max_gap[0])):
    #         max_gap[0] = gap_start
    #         max_gap[1] = gap_start + gap_width
    #         found_gap = True

    #     gap_width += 1
        
    # if(found_gap):
    #     return max_gap
    # else:
    #     return None
    start_i,end_i = 0, 0
    best_start,best_end = 0, 0

    for i in range(len(free_space_ranges)):
        if free_space_ranges[i] > 5.0:
            end_i += 1
        else:
            if end_i - start_i > best_end - best_start:
                best_start, best_end = start_i, end_i
            
            start_i = i + 1
            end_i = i

    if end_i - start_i > best_end - best_start:
        best_start = start_i
        best_end   = end_i
    
    if(best_end >= best_start + 2):
        return best_start, best_end
    else:
        return None, None

def main(args=None):
    print(find_max_gap(np.array([0.5, 5.1, 6.0, 7.0, 99999, 3.0, 99999, 3.0, 99999, 8.0, 1.0, 3.0])))
    print(find_max_gap(np.array([5, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0])))

if __name__ == '__main__':
    main()