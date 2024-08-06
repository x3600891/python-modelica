import numpy as np
import matplotlib.pyplot as plt
import pandas as pd
from dymola.dymola_interface import DymolaInterface
class GeneticAlgorithm:
    def __init__(self, objective_function, bounds, population_size=5, generations=10, crossover_rate=0.8, mutation_rate=0.2):
        self.objective_function = objective_function
        self.bounds = bounds
        self.population_size = population_size
        self.generations = generations
        self.crossover_rate = crossover_rate
        self.mutation_rate = mutation_rate
        self.best_fitnesses = []

    def initialize_population(self):
        population = []
        for _ in range(self.population_size):
            x = np.random.uniform(low=self.bounds[0][0], high=self.bounds[0][1])
            y = np.random.uniform(low=self.bounds[1][0], high=self.bounds[1][1])
            individual = np.array([x, y])
            population.append(individual)
        return np.array(population)

    def evaluate_population(self, population):
        fitness_values = []
        for individual in population:
            fitness_values.append(self.objective_function(*individual))
        return np.array(fitness_values)

    def select_parents(self, population, fitness_values):
        sorted_indices = np.argsort(fitness_values)
        sorted_population = population[sorted_indices]
        selected_parents = sorted_population[:self.population_size // 2]
        return selected_parents
    def crossover(self, parents):
        children = []
        for i in range(len(parents) // 2):
            parent1 = parents[i]
            parent2 = parents[len(parents) - i - 1]
            child1 = np.copy(parent1)
            child2 = np.copy(parent2)
        if np.random.rand() < self.crossover_rate:
            crossover_point = np.random.randint(1, len(parent1))  # 修正交叉点的选取范围
            child1[crossover_point:] = parent2[crossover_point:]
            child2[crossover_point:] = parent1[crossover_point:]
        children.append(child1)
        children.append(child2)
        return np.array(children)



    def mutate(self, population):
        for i in range(len(population)):
            if np.random.rand() < self.mutation_rate:
                mutation_point = np.random.randint(0, len(population[i]))
                population[i][mutation_point] = np.random.uniform(low=self.bounds[mutation_point][0], high=self.bounds[mutation_point][1])
        return population

    def optimize(self):
        population = self.initialize_population()
        for generation in range(self.generations):
            fitness_values = self.evaluate_population(population)
            best_fitness = np.min(fitness_values)
            self.best_fitnesses.append(best_fitness)
            parents = self.select_parents(population, fitness_values)
            offspring = self.crossover(parents)
            population = np.concatenate((parents, offspring))
            population = self.mutate(population)
        best_solution_index = np.argmin(self.evaluate_population(population))
        best_solution = population[best_solution_index]
        best_fitness = self.objective_function(*best_solution)
        return best_solution, best_fitness

    def plot_fitness(self):
        plt.plot(range(1, self.generations + 1), self.best_fitnesses)
        plt.xlabel('Generation')
        plt.ylabel('Best Fitness')
        plt.title('Evolution of Best Fitness')
        plt.show()


#自己编写目标函数
def create_objective_function(dymola):
    def objective_function(x,y): 
        file="res" 
        x=str(x)
        y=str(y)      
        dymola.simulateModel("simulationAndOpt.system",resultFile=file)
        dymola.ExecuteCommand(f"x={x};y={y};")
        dymola.simulateModel("simulationAndOpt.system",resultFile=file)
        signals = dymola.readTrajectoryNames(fileName=f"{file}.mat")
        size=dymola.readTrajectorySize(f"{file}.mat")
        result = pd.DataFrame(np.array(dymola.readTrajectory(fileName=f"{file}.mat",signals=signals,rows=size)).transpose(),columns=signals)
        return result.loc[0,'energy']
    return objective_function
    

# 示例：定义目标函数
# def objective_function_raw(x, y):
#     model_path="E:/111jieping/fangzhenjiyouhua/"
#     mod=ModelicaSystem(model_path + "simulationAndOpt.mo","simulationAndOpt.system")

#     mod.setParameters(["x="+str(x),"y="+str(y)])

#     mod.buildModel()

#     mod.setSimulationOptions(["stopTime=0.2","tolerance=1e-08"])
#     mod.simulate(resultfile="result.mat")
#     result=mod.getSolutions(["time","energy"])

    return result[1][0]

# 示例：定义参数范围约束
bounds = [(-10, 10), (-10, 10)]  # 参数 x 的范围约束为 -5 到 5，参数 y 的范围约束为 -3 到 3

path=r"D:/Program Files/Dymola 2023/bin64/Dymola.exe"
dymola = DymolaInterface(path,showwindow=True,port=1)
dymola.openModel(path="F:/00 Study/03 Github/python-modelica/simulationAndOpt.mo")
objective_function=create_objective_function(dymola)

# 创建遗传算法实例
ga = GeneticAlgorithm(objective_function, bounds)

# 运行优化算法
best_solution, best_fitness = ga.optimize()

print("最优解:", best_solution)
print("最优解对应的目标函数值:", best_fitness)

# 绘制目标值随优化次数的变化趋势图
ga.plot_fitness()


