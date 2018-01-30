import pandas as pd

layer = pd.DataFrame([], columns=("Category", "Subcategory", "height", "width"))

layer = layer.append({"Category": 'line'}, ignore_index=True)
layer = layer.append({"Category": 'Polygon'}, ignore_index=True)
print(layer)