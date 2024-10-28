import math
import numpy as np
import matplotlib.pyplot as plt
import matplotlib as mpl
import matplotlib as mpl
from matplotlib.colors import ListedColormap
from mpl_toolkits.axes_grid1 import make_axes_locatable
import seaborn as sns
from mip import *
import pandas as pd



class GLOSIP(object): 

    def __init__(self, configuration):
        #Read configuration from input file (JSON format expected)
        self.T      = configuration['T']
        self.D      = [{'id': str(i*self.T[1]+j), 'position':[i,j]} for i, row in enumerate(configuration['D']) for j, drone in enumerate(row) if drone == 1]
        self.P      = np.array(configuration['P']) - 1
        self.M      = configuration['M']
        self.H      = configuration['H']
        self.H = np.array([[self.H[i][drone['position'][0]][drone['position'][1]] for j, drone in enumerate(self.D)] for i, microservice in enumerate(self.M)])
        

        # C is the Big-M constant, it is a value outside the domain of all parameters
        self.C      = (np.array(self.P).max() * np.array(self.H).max()) + 1
        print('Parameters loaded from configuration!')
        # Initialize the model
        self.model = Model(sense=MINIMIZE, solver_name=GRB)
        self.model.verbose = 1
        self.model.threads = -1
        #self.model.max_mip_gap = 0.001
        self.model.emphasis = 1


    def showDeploymentMatrix(self, background_img: str = None):
        cmap = ListedColormap(['#444444', '#FFFFFF'], name='from_list', N=None)
        deployment_matrix = np.zeros((self.T[0], self.T[1]), dtype=np.int8)
        for drone in self.D:
            deployment_matrix[drone['position'][0]][drone['position'][1]] = 1
            
        sns.heatmap(deployment_matrix, cmap=cmap, linecolor='gainsboro', linewidths=.1, alpha=0.5)
        print(background_img)
        plt.imshow(plt.imread(background_img), extent=[0, self.T[1], self.T[0], 0])

        plt.show() 

    def showHeatMaps(self, background_img: str = None) :
        rows = math.ceil(len(self.H[:])/2)
        fig, ax = plt.subplots(rows, 2)
        heatmaps = np.zeros((len(self.M), self.T[0], self.T[1]), dtype=np.int8)
        for i, ms_heat in enumerate(self.H):
            for j, drone in enumerate(self.D):
                heatmaps[i][drone['position'][0]][drone['position'][1]] = ms_heat[j]
        
        for i, (heatmap, subplot) in enumerate(zip(heatmaps, np.array(ax).reshape((-1)))):
            subplot.set_title(self.M[i], fontweight='bold', size=20)
            subplot.tick_params(axis='both', which='major', labelsize=16)
            sns_heatmap = sns.heatmap(heatmap, cmap="hot", vmin = 0, vmax= 5, ax=subplot, linecolor='gainsboro', linewidths=.1, alpha=0.5)
            cax = sns_heatmap.figure.axes[-1].tick_params(labelsize=16)
            try:
                ax[i//2][i%2].imshow(plt.imread(background_img), extent=[0, self.T[1], self.T[0], 0])
            except:
                pass
        if (len(self.H[:]) % 2 == 1): fig.delaxes(ax[-1][-1])
        plt.show()


    def initializeModel(self):
        #Set the decision variables
        for microservice in self.M:
            for drone in self.D:
                    self.model.add_var(f"x_{microservice},d{drone['position']}", var_type=BINARY)
        decision_variables = np.array(self.model.vars)

        decision_variables_aux = decision_variables.reshape((len(self.M), -1)).T # 

        for i, x in enumerate(decision_variables_aux):   
            constraint = xsum(x) <= 1
            self.model.add_constr(constraint, name=f'Capacity of d_{self.D[i]["position"]} not surpased')

        decision_variables_aux = decision_variables.reshape((len(self.M),-1))
        
        for i, x in enumerate(decision_variables_aux):   
            constraint = xsum(x) == 1
            self.model.add_constr(constraint, name=f'Microservice {self.M[i]} deployed')

        decision_variables_aux = decision_variables.reshape((len(self.M), len(self.D)))
        objective_function = 0
        objective_function = xsum([decision_variables_aux[k][j] * self.H[k][i] * self.P[self.D[i]['position'][0]*self.T[1] + self.D[i]['position'][1]][self.D[j]['position'][0]*self.T[1] + self.D[j]['position'][1]] for i, drone1 in enumerate(self.D) for j, drone2 in enumerate(self.D) for k, ms in enumerate(self.M)])

        self.model.objective = minimize(objective_function)

    def solve(self):
        self.model.optimize()
        return pd.DataFrame([{"Decision Variable": var.name, "Value": var.x} for var in self.model.vars])

    def printSolution(self, background_img: str = None):
        solution_df = pd.DataFrame([{"Decision Variable": var.name, "Value": var.x} for var in self.model.vars[:]])

        print(solution_df)
        solution_ndarray = solution_df.to_numpy()[:,1]
        print(solution_ndarray.shape)
        solution_ndarray = solution_ndarray.reshape(len(self.M), len(self.D), -1)[:,:,0]
        deployment_matrix = np.zeros((len(self.M), self.T[0], self.T[1]))
        for microservice in range(len(self.M)):
            for i, drone in enumerate(self.D):
               deployment_matrix[microservice][drone['position'][0]][drone['position'][1]] =  solution_ndarray[microservice][i]


        deployment_matrix = [m *(i+1) for m, i in zip(deployment_matrix[:], range(deployment_matrix.shape[0]))]
        deployment_matrix = np.add.reduce(deployment_matrix[:] )

        colors = ['#FFFFFF', '#FF2D01','#0184FF', '#FFBA01','#B601FF', '#FFF701', '#9BFF01', '#01FFDC','#010DFF', '#FF01E0']
        cmap = ListedColormap(colors[0:len(self.M)+1], name='from_list', N=None)

        plt.figure(figsize = (24,8))
        ax = sns.heatmap(deployment_matrix.astype(np.int8), cmap=cmap, linecolor='gray', linewidths=.3, alpha=0.6)
        plt.title('Matriz de despliegue', fontweight='bold', size=20)

        colorbar = ax.collections[0].colorbar
        n = len(self.M) + 1


        r = colorbar.vmax - colorbar.vmin
        colorbar.set_ticks([colorbar.vmin + 0.5 * r / (n) + r * i / (n) for i in range(len(self.M) + 1)])
        colorbar.set_ticklabels(['Vacío'] + self.M)
        colorbar.ax.tick_params(axis= 'both', labelsize = 18)
        try:
            plt.imshow(plt.imread(background_img), extent=[0, self.T[1], self.T[0], 0])
        except:
            pass
        plt.show()

    def getMin(self, microservice_index, src_drone, proposed_solution):
        hops_to_dsts = []
        i  = src_drone['position'][0]
        j  = src_drone['position'][1]
        for dst_drone_index, dst_drone in enumerate(self.D):
                if (proposed_solution[microservice_index][dst_drone_index] == 1 ):
                    ii = dst_drone['position'][0]
                    jj = dst_drone['position'][1]
                    hops_to_dsts.append(self.P[i*self.T[1]+j][ii*self.T[1]+jj])
        print(f'{self.M[microservice_index]} - d({i},{j}) ->', hops_to_dsts)
        return min(hops_to_dsts)
        
    def solutionLongFormat(self, scenario_name: str):

        proposed_solution = np.array([decision_variable.x for decision_variable in self.model.vars[:]])

        # print(len(proposed_solution))
        proposed_solution_aux = proposed_solution.reshape((len(self.M), len(self.D), -1))[:,:,0]

        # Calcular el número de saltos mínimo que hay que dar para, desde un dron de origen, poder comsumir un servicio
        # alojado en el dron de destino más cercano
        proposed_solution = []
        for ms in range(len(self.M)):
            for i ,d in enumerate(self.D): 
                if (self.H[ms][i] > 0):
                   proposed_solution.append(self.getMin(ms, d, proposed_solution_aux))
                else:
                    proposed_solution.append(-1)
                        
                       
        proposed_solution = np.array(proposed_solution).reshape((len(self.M), -1))
        proposed_solution = proposed_solution.tolist()
        
        for index, sublist in enumerate(proposed_solution):
            proposed_solution[index] = [element for element in sublist if element != -1]
          
        dataset = []
        for ms, sublist in zip(self.M, proposed_solution):
                for cost in sublist:
                    dataset.append([scenario_name, ms, cost])

        return dataset
    
    
    def solutionToCSV(self, output_file: str = 'out.csv'):
        decision_variables = np.array([decision_variable.x for decision_variable in self.model.vars[:]])
        decision_variables = decision_variables.reshape((len(self.M), len(self.D), -1))[:,:,0].T #drone -> Xms1, Xms2, Xms3, ..., Xmsn
        heatmaps = self.H.T # drone -> Hms1, Hms2, Hms3 ... Hmsn
        jumps = np.array([[ self.getMin(ms, d, decision_variables.T) if (self.H[ms][i] > 0) else -1 for i, d in enumerate(self.D)] for ms in range(len(self.M))]).T #drone -> ms1, ms2, ms3, ..., msn
        adjacencyList = np.array([[ 1 if max( abs(d2['position'][0] - d['position'][0]), abs(d2['position'][1] - d['position'][1])) < 2 else 0  for j, d2 in enumerate(self.D)] for i, d in enumerate(self.D)]) #drone -> drone, drone2, drone3, ..., drone4
        print(decision_variables.shape, heatmaps.shape, jumps.shape, adjacencyList.shape)
        allLists = zip(decision_variables, heatmaps, jumps, adjacencyList)
        result_list = []        
        for i, (Xms, Hms, Jd2, Ad2) in enumerate(allLists):
            result_list.append(
                {
                   'drone': f"drone_{self.D[i]['position'][0]}-{self.D[i]['position'][1]}",
                })
            print(Xms.shape, Hms.shape, Jd2.shape, Ad2.shape)
            for j, (x, h, ju) in enumerate(zip(Xms, Hms, Jd2)):
                result_list[i][self.M[j]] = x
                result_list[i][f'heat_{self.M[j]}'] = h
                result_list[i][f'jumps_{self.M[j]}'] = ju
            for k, drone in enumerate(Ad2):
                result_list[i][f'adj_drone{k}'] = drone
        
        column_list = ['drone']
        [column_list.append(m) for m in self.M]
        [column_list.append(f'heat_{ms}') for ms in self.M]
        [column_list.append(f'jumps_{self.M[i]}') for i in range(len(self.M))]
        [column_list.append(f'adj_drone{i}') for i in range(len(self.D))]
        result_df = pd.DataFrame(result_list, columns=column_list)
        result_df.to_csv(output_file, sep=',', index=False)
        
    
    def resetModel(self):
        self.model  = Model(solver_name='CBC')


    def flatten(self, xs):
        result = []
        if isinstance(xs, (list, tuple)):
            for x in xs:
                result.extend(self.flatten(x))
        else:
            result.append(xs)
        return result
    


from tkinter import Tk
from tkinter import filedialog
import json
import time

configuration = json.load(open(r'./test_001.json'))

my_model = GLOSIP(configuration=configuration)
#my_model.showDeploymentMatrix(r'./scenario_background.png')
my_model.showHeatMaps(r'./scenario_background.png')
my_model.initializeModel()
start_time = time.time()
my_model.solve()
print("--- %s seconds ---" % (time.time() - start_time))

my_model.printSolution()

scenarios = my_model.solutionLongFormat('Unevenly distributed scenario with focal points')
my_model.solutionToCSV('test_002.csv')


configuration = json.load(open(r'./test_002.json'))

my_model = GLOSIP(configuration=configuration)
my_model.showHeatMaps(r'./scenario_background.png')
my_model.initializeModel()
start_time = time.time()
my_model.solve()
print("--- %s seconds ---" % (time.time() - start_time))

my_model.printSolution()

scenarios += my_model.solutionLongFormat('Evenly balanced scenario')
my_model.solutionToCSV('test_001.csv')

configuration = json.load(open(r'./test_000.json'))

my_model = GLOSIP(configuration=configuration)
my_model.showHeatMaps(r'./scenario_background.png')
my_model.initializeModel()
start_time = time.time()
my_model.solve()
print("--- %s seconds ---" % (time.time() - start_time))

my_model.printSolution()

scenarios += my_model.solutionLongFormat('Highly skewed scenario')
my_model.solutionToCSV('test_000.csv')



df = pd.DataFrame(data = scenarios,
                  columns = ['Scenario','Microservice', 'Hops'])




medianprops = {'linestyle':'-', 'linewidth':4, 'color':'darkorange'}
meanpointprops = {'marker':'D', 'markeredgecolor':'indigo','markerfacecolor':'indigo'}

sns.set_theme(style="ticks", palette="pastel")


fig, ax = plt.subplots(figsize=(20, 10))
my_pal = {'Unevenly distributed scenario with focal points': 'g', 'Evenly balanced scenario': 'b', 'Highly skewed scenario': 'r'}


ax = sns.boxplot(data= df, x="Microservice", y="Hops", hue="Scenario", palette=my_pal, 
            meanprops = meanpointprops, showmeans=True, 
            flierprops={"marker": "x"},
            medianprops=medianprops)


hatches = ['/', '.', 'x']
# select the correct patches
patches = [patch for patch in ax.patches if type(patch) == mpl.patches.PathPatch]
# the number of patches should be evenly divisible by the number of hatches
h = hatches * (len(patches) // len(hatches))
# iterate through the patches for each subplot
for patch, hatch in zip(patches, h):
    patch.set_hatch(hatch)
    patch.set_edgecolor('black')
    
l = ax.legend(fontsize=20)
for lp, hatch in zip(l.get_patches(), hatches):
    lp.set_hatch(hatch)
    lp.set_edgecolor('black')
    



plt.xticks(range(len(my_model.M)), my_model.M, rotation=45, fontsize=18)
plt.yticks(range(0, int(df.loc[df['Hops'].idxmax()]['Hops'])+2), fontsize=18)
plt.xlabel('Microservices', fontsize=20)
plt.ylabel('Nº hops', fontsize=20)

plt.title('Hops needed to serve the microservices to hosts', pad=30, fontsize=28, fontweight='bold')

plt.show()
