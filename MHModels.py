import keras
from keras.models import Sequential
from keras.layers import Dense

def buildModel():
    model = Sequential()
    model.add(Dense(6, activation='tanh', input_shape=(16,)))
    model.add(Dense(21))
    return model

def saveModels(models, name):
    '''
        Saves a list of models.
        models: list of models to save
        name: common name for all models
    '''
    for i in range(len(models)):
        models[i].save(name+str(i))
    print('Models saved Successfully!')

def loadModels(name, no=20):
    '''
        Loads a bunch of models and returns them as a list
        name: common name for all models
        no: no of models to load
    '''
    models = []
    for i in range(no):
        models.append(keras.models.load_model(name+str(i)))
    print('Models loaded successfully!')
    return models