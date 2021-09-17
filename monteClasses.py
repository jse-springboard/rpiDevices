'''
Monte Carlo simulator support classes
Author: Jordan Eriksen
Date: 2021-09-17
'''
import sys, os
import numpy as np
import pandas as pd
from scipy.stats import norm
import datetime

class dimension:
    def __init__(self,nominal,tolerance=0,description='Undefined',n=1000000):
        self.mean = nominal
        self.std = tolerance/3
        self.description = description
        self.n = n
        self._calcSigma()
        self._simulate()
    
    def _calcSigma(self):
        self.sigma = {
            1:(self.mean - 1*self.std,self.mean + 1*self.std),
            2:(self.mean - 2*self.std,self.mean + 2*self.std),
            3:(self.mean - 3*self.std,self.mean + 3*self.std),
            6:(self.mean - 6*self.std,self.mean + 6*self.std)
        }
        
    def _simulate(self):
        '''
        Return distribution of results assuming symmetric tolerances
        '''
        self.distribution = np.random.normal(self.mean,self.std,self.n)

    def sigTable(self):
        '''
        Output table of means and sigma1,2,3 for the final stack and the subcomponents.
        '''
        self.output_table = {}
        self.output_table[self.description] = {
                '-6 sig':self.sigma[6][0],
                '-3 sig':self.sigma[3][0],
                '-2 sig':self.sigma[2][0],
                '-1 sig':self.sigma[1][0],
                'mean':self.mean,
                '+1 sig':self.sigma[1][1],
                '+2 sig':self.sigma[2][1],
                '+3 sig':self.sigma[3][1],
                '+6 sig':self.sigma[6][1]}
        
        self.output_DF = pd.DataFrame(self.output_table).transpose().round(3)
        self.output_DF = self.output_DF[['-6 sig','-3 sig','-2 sig','-1 sig','mean','+1 sig','+2 sig','+3 sig','+6 sig']]

        return self.output_DF
    
    def save(self,name):
        '''
        Save output DataFrame as pickle file
        '''

        if not self.output_DF:
            self.simulate()

        self.output_DF.to_pickle(f'./Output/{datetime.datetime.now().date()} {name}.pkl')

class stack:
    '''
    Sort combine and manage the stack
    '''
    def __init__(self):
        self.dimensions = {}
        self.descList = []
        self.sigTable = pd.DataFrame(columns=['-6 sig','-3 sig','-2 sig','-1 sig','mean','+1 sig','+2 sig','+3 sig','+6 sig'])
        self.stackTable = pd.DataFrame(columns=['Description','Sign','Mean','Tolerance'])

    def _new(self,dim,sign):
        '''
        Updates stack data after add or sub
        '''
        self.dimensions[dim.description] = dim
        self.descList.append((dim.description,sign))

        if sign == '+':
            self.distribution = self.stack + dim.distribution
        elif sign == '-':
            self.distribution = self.stack - dim.distribution

        self.sigTable = self.sigTable.append(dim.sigTable)
        self.stackTable = self.stackTable.append({
            'Description':dim.description,
            'Sign':sign,
            'Nominal':dim.mean,
            'Tolerance':dim.std*3
            })

    def add(self, dim):
        '''
        Add dimension to the stack
        '''
        self._new(dim,'+')

    def sub(self,dim):
        '''
        Subtract dimension from the stack
        '''
        self._new(dim,'-')

    def rm(self):
        '''
        Request user input to determine which entry in the stack to remove
        '''
        print('Select dimensions to remove:')
        num = 0
        for i in self.dimensions:
            num += 1
            print(f' [{num}] -> {i} ')
        print('')

        rmed = input(f'Variable(s) to remove? [list | int]: ')
        itemsRemove = [int(i) for i in list(rmed) if i not in [',','[',']',' ']]

        for i in itemsRemove:
            self.dimensions.pop(self.descList[i])

    def combine(self):
        '''
        Combine constituent dimensions to build tolerance stack
        '''
