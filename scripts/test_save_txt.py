import os
# Example M by N array
M = 3
N = 4
array = [[1, 2, 3, 4],
         [5, 6, 7, 8],
         [9, 10, 11, 12]]
i=5

# Specify the folder name
folder_name1 = "Data/data_aubo"
folder_name2 = "Data/data_leika"

# Create the folder
os.makedirs(folder_name1, exist_ok=True)
os.makedirs(folder_name2, exist_ok=True)

# Save the array to a text file
with open('Data/data_aubo/array'+str(i)+'.txt', 'w') as file:
    for row in array:
        file.write(', '.join(map(str, row)) + '\n')

# Save the array to a text file
with open('Data/data_leika/array'+str(i)+'.txt', 'w') as file:
    for row in array:
        file.write(', '.join(map(str, row)) + '\n')