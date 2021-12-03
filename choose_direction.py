import numpy as np
import sys

# Options
size = 10
window_size = 3
stride = 2  # default 1

# Input: time to collision array
ttc = np.random.rand(size, size)

# Window sliding setup
height, width = ttc.shape
convs_heights = np.arange(int(height - window_size + 1))[::stride]
convs_widths = np.arange(int(width - window_size + 1))[::stride]
best_pos = (0, 0)
best_window_ttc = 0

# Slide through grid windows
for i in convs_heights: 
  for j in convs_widths: 

    # Get closest pixel
    window = ttc[i:i+window_size,j:j+window_size]
    cur_min = np.min(window)

    # Want farthest window where all pixels are farther than previous farthest
    if cur_min > best_window_ttc: 
      best_window_ttc = cur_min
      best_pos = (i, j)


np.set_printoptions(linewidth=np.inf, threshold=sys.maxsize)
print('best_pos:', best_pos)
print('ttc grid:\n', ttc)

