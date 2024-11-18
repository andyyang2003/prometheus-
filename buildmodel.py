#using logistic regression on lab fire dataset to detect fire
#11.17 andy yang - using carton_1.csv as initial dataset

import pandas as pd
from sklearn.model_selection import train_test_split
from sklearn.linear_model import LogisticRegression 
from sklearn import metrics
df = pd.read_csv('carton_1.csv')
print(df.describe())
X = df.drop(columns=['Reading ID','Time','Detector', 'Status'])
Y = df.Detector
X_train, X_test, Y_train, Y_test = train_test_split(X, Y, train_size=0.7, test_size=0.30, random_state=42) #42 is arbitrary its literlaly because its a joke
model = LogisticRegression(random_state = 42).fit(X_train, Y_train)
Y_pred = model.predict(X_test)
accuracy = metrics.accuracy_score(Y_test, Y_pred)
print(accuracy)

new_data = pd.DataFrame({
    'Humidity' : [44.9],
    'Temperature' : [21.3],
    'MQ139' : [173],
    'TVOC' : [2787],
    'eCO2' : [4751] 
})

predicted = model.predict(new_data)
print(predicted)