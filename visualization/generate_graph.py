import statistics

#################################### symbols ####################################
# universal factor graph
def AName():
    return "A"

def BName():
    return "B"

def ABName():
    return "AB"

def CName():
    return "C"

def stateName(i):
    return "x{}".format(i)

def actionName(i):
    return "u{}".format(i)

def observationName(i):
    return "y{}".format(i)

def ASymbol():
    return "A"

def BSymbol():
    return "B"

def ABSymbol():
    return "A,B"

def CSymbol():
    return "C"

def stateSymbol(i):
    return "x_{}".format(i)

def actionSymbol(i):
    return "u_{}".format(i)

def observationSymbol(i):
    return "y_{}".format(i)

def transitionFactorName(i):
    return "trans{}".format(i)

def observationFactorName(i):
    return "obs{}".format(i)

def stateCostFactorName(i):
    return "costX{}".format(i)

def actionCostFactorName(i):
    return "costU{}".format(i)

# kinodynamics graph
def poseName(i):
    return "p{}".format(i)

def twistName(i):
    return "V{}".format(i)

def twistAccelName(i):
    return "A{}".format(i)

def angleName(j):
    return "q{}".format(j)

def angularVelName(j):
    return "v{}".format(j)

def angularAccelName(j):
    return "a{}".format(j)

def torqueName(j):
    return "t{}".format(j)

def wrenchName(i, j):
    return "F{}{}".format(i, j)

def poseSymbol(i):
    return "T_{}".format(i)

def twistSymbol(i):
    return "\\mathcal{{V}}_{}".format(i)

def twistAccelSymbol(i):
    return "\\dot{{\\mathcal{{V}}}}_{}".format(i)

def angleSymbol(j):
    return "\\theta_{}".format(j)

def angularVelSymbol(j):
    return "\\dot{{\\theta}}_{}".format(j)

def angularAccelSymbol(j):
    return "\\ddot{{\\theta}}_{}".format(j)

def torqueSymbol(j):
    return "\\tau_{}".format(j)

def wrenchSymbol(i, j):
    return "\\mathcal{{F}}_{{{}{}}}".format(i, j)

def poseFactorName(j):
    return "fq{}".format(j)

def twistFactorName(j):
    return "fv{}".format(j)

def accelFactorName(j):
    return "fa{}".format(j)

def wrenchEqFactorName(j):
    return "fweq{}".format(j)

def torqueFactorName(j):
    return "ft{}".format(j)

def dynamicsFactorName(i):
    return "fd{}".format(i)

def priorFactorName(i):
    return "prior{}".format(i)

#################################### FG classes ####################################
class Variable:
    def __init__(self, name, symbol, loc, color="white", fixed=False):
        self.name = name
        self.symbol = symbol
        self.loc = list(loc)
        self.color = color
        self.fixed = fixed

class Factor:
    def __init__(self, name, variables, loc, color="black"):
        self.name = name
        self.eqn = None
        self.variables = list(variables)
        self.loc = list(loc)
        self.color = color

class Graph:
    def __init__(self):
        self.variables = {}
        self.factors = []

    def addVaraible(self, variable):
        self.variables[variable.name] = variable

    def addFactor(self, name, var_names, loc=None, color="black"):
        var_list = []
        for var_name in var_names:
            var_list.append(self.variables[var_name])
        
        if loc is None:
            xs = []
            ys = []
            for variable in var_list:
                xs.append(variable.loc[0])
                ys.append(variable.loc[1])
            loc = [statistics.mean(xs), statistics.mean(ys)]

        self.factors.append(Factor(name, var_list, loc, color=color))
    
    def generateLatexScript(self, file_path):
        f = open(file_path, "w")

        f.write("\\begin{tikzpicture}\n")
        # set locations
        for variable in self.variables.values():
            f.write("\\coordinate ({}) at ({}, {});\n".format(variable.name, variable.loc[0], variable.loc[1]))

        for factor in self.factors:
            f.write("\\coordinate ({}) at ({}, {});\n".format(factor.name, factor.loc[0], factor.loc[1]))

        for factor in self.factors:
            for variable in factor.variables:
                f.write("\\path[draw, line width=0.5pt] ({}) -- ({});\n".format(variable.name, factor.name))
        
        for variable in self.variables.values():
            if variable.fixed:
                f.write("\\node[scale=1, fill={}][rectangle, inner sep=2.8pt, draw] ({}) at ({}, {}) {{${}$}};\n".format(variable.color, variable.name, variable.loc[0], variable.loc[1], variable.symbol))
            else:
                f.write("\\node[scale=1, fill={}][circle, inner sep=2.8pt, draw] ({}) at ({}, {}) {{${}$}};\n".format(variable.color, variable.name, variable.loc[0], variable.loc[1], variable.symbol))

        for factor in self.factors:
            f.write("\\draw [{}, fill={}] ({}) circle (0.05);\n".format(factor.color, factor.color, factor.name))

        f.write("\\end{tikzpicture}\n")
        f.close()


#################################### locations ####################################
def gridLocation(x, y):
    return [x*3, -y*2]


#################################### scenarios ####################################
def jumpingRobotFG():
    num_links = 6
    graph = Graph()
    
    pose_h = 1
    twist_h = 2
    twistAcc_h = 3

    color_ground = "red"
    color_air = "blue"

    for i in range(num_links):
        fixed = (i==0)
        color = color_ground if fixed else "white"
        graph.addVaraible(Variable(poseName(i), poseSymbol(i), gridLocation(i, pose_h), fixed=fixed, color=color))
        graph.addVaraible(Variable(twistName(i), twistSymbol(i), gridLocation(i, twist_h), fixed=fixed, color=color))
        graph.addVaraible(Variable(twistAccelName(i), twistAccelSymbol(i), gridLocation(i, twistAcc_h), fixed=fixed, color=color))
    graph.addVaraible(Variable(poseName(num_links), poseSymbol(0), gridLocation(num_links, pose_h), fixed=True, color=color_ground))
    graph.addVaraible(Variable(twistName(num_links), twistSymbol(0), gridLocation(num_links, twist_h), fixed=True, color=color_ground))
    graph.addVaraible(Variable(twistAccelName(num_links), twistAccelSymbol(0), gridLocation(num_links, twistAcc_h), fixed=True, color=color_ground))

    for j in range(1, num_links+1):
        color = color_ground if (j==1) or (j==num_links) else "white"
        graph.addVaraible(Variable(angleName(j), angleSymbol(j), gridLocation(j - 0.5, 1.5), color=color))
        graph.addVaraible(Variable(angularVelName(j), angularVelSymbol(j), gridLocation(j - 0.5, 2.5), color=color))
        graph.addVaraible(Variable(angularAccelName(j), angularAccelSymbol(j), gridLocation(j - 0.5, 3.5), color=color))
        graph.addVaraible(Variable(torqueName(j), torqueSymbol(j), gridLocation(j - 0.5, 4.5), color=color))
        i1 = j - 1
        i2 = j
        i2_symbol = j % num_links
        
        color1 = color_ground if (i1==0) or (i1==num_links) else "white"
        color2 = color_ground if (i2==0) or (i2==num_links) else "white"
        graph.addVaraible(Variable(wrenchName(i1, j), wrenchSymbol(i1, j), gridLocation(j - 0.75, 4), color=color1))
        graph.addVaraible(Variable(wrenchName(i2, j), wrenchSymbol(i2_symbol, j), gridLocation(j - 0.25, 4), color=color2))

    for i in range(1, num_links):
        j1 = i
        j2 = i + 1
        graph.addFactor(dynamicsFactorName(i), [twistAccelName(i), twistName(i), poseName(i), wrenchName(i, j1), wrenchName(i, j2)], gridLocation(i, 3.5))


    for j in range(1, num_links+1):
        color = color_ground if (j==1) or (j==num_links) else "black"
        i1 = j-1
        i2 = j
        graph.addFactor(poseFactorName(j), [poseName(i1), poseName(i2), angleName(j)], color=color)
        graph.addFactor(twistFactorName(j), [twistName(i1), twistName(i2), angularVelName(j), angleName(j)], gridLocation(j-0.5, 2.25), color=color)
        graph.addFactor(accelFactorName(j), [twistAccelName(i1), twistAccelName(i2), twistName(i2), angularAccelName(j), angularVelName(j), angleName(j)], gridLocation(j-0.5, 3.25), color=color)
        graph.addFactor(wrenchEqFactorName(j), [wrenchName(i1, j), wrenchName(i2, j), angleName(j)], gridLocation(j-0.5, 4), color=color)
        graph.addFactor(torqueFactorName(j), [torqueName(j), wrenchName(i2, j)], color=color)

    graph.addFactor(priorFactorName(0), [wrenchName(1, 1)], gridLocation(0.75, 4.5), color=color_air)
    graph.addFactor(priorFactorName(5), [wrenchName(5, 6)], gridLocation(5.25, 4.5), color=color_air)

    graph.generateLatexScript("jumping_robot.tikz")

def serialFourBarFG():
    num_links = 4

    q_color = "red"
    v_color = "yellow"
    a_color = "green"
    d_color = "cyan"
    graph = Graph()
    for i in range(num_links):
        graph.addVaraible(Variable(poseName(i), poseSymbol(i), gridLocation(i, 1), color=q_color))
        graph.addVaraible(Variable(twistName(i), twistSymbol(i), gridLocation(i, 2), color=v_color))
        graph.addVaraible(Variable(twistAccelName(i), twistAccelSymbol(i), gridLocation(i, 3), color=a_color))

    for j in range(1, num_links):
        graph.addVaraible(Variable(angleName(j), angleSymbol(j), gridLocation(j - 0.5, 1.5), color=q_color))
        graph.addVaraible(Variable(angularVelName(j), angularVelSymbol(j), gridLocation(j - 0.5, 2.5), color=v_color))
        graph.addVaraible(Variable(angularAccelName(j), angularAccelSymbol(j), gridLocation(j - 0.5, 3.5), color=a_color))
        graph.addVaraible(Variable(torqueName(j), torqueSymbol(j), gridLocation(j - 0.5, 4.5), color=d_color))
        i1 = j - 1
        i2 = j % num_links
        graph.addVaraible(Variable(wrenchName(i1, j), wrenchSymbol(i1, j), gridLocation(j - 0.75, 4), color=d_color))
        graph.addVaraible(Variable(wrenchName(i2, j), wrenchSymbol(i2, j), gridLocation(j - 0.25, 4), color=d_color))

    for i in range(num_links):
        if i>0:
            j1 = i
            j2 = i + 1
            if i<num_links-1:
                graph.addFactor(dynamicsFactorName(i), [twistAccelName(i), twistName(i), poseName(i), wrenchName(i, j1), wrenchName(i, j2)], gridLocation(i, 3.5), color=d_color)
            else:
                graph.addFactor(dynamicsFactorName(i), [twistAccelName(i), twistName(i), poseName(i), wrenchName(i, j1)], gridLocation(i, 3.5), color=d_color)


    for j in range(1, num_links):
        i1 = j-1
        i2 = j % num_links
        graph.addFactor(poseFactorName(j), [poseName(i1), poseName(i2), angleName(j)], color=q_color)
        graph.addFactor(twistFactorName(j), [twistName(i1), twistName(i2), angularVelName(j), angleName(j)], gridLocation(j-0.5, 2.25), color=v_color)
        graph.addFactor(accelFactorName(j), [twistAccelName(i1), twistAccelName(i2), twistName(i2), angularAccelName(j), angularVelName(j), angleName(j)], gridLocation(j-0.5, 3.25), color=a_color)
        graph.addFactor(wrenchEqFactorName(j), [wrenchName(i1, j), wrenchName(i2, j), angleName(j)], gridLocation(j-0.5, 4), color=d_color)
        graph.addFactor(torqueFactorName(j), [torqueName(j), wrenchName(i2, j)], color=d_color)

    graph.generateLatexScript("serial_four_bar.tikz")


def simplifiedFourBarFG():
    num_links = 4

    q_color = "black"
    v_color = "black"
    a_color = "green"
    d_color = "cyan"
    graph = Graph()
    for i in range(num_links):
        if i==0:
            graph.addVaraible(Variable(twistAccelName(i), twistAccelSymbol(i), gridLocation(i, 1), color=a_color, fixed=True))
        else:
            graph.addVaraible(Variable(twistAccelName(i), twistAccelSymbol(i), gridLocation(i, 1), color=a_color))
    graph.addVaraible(Variable(twistAccelName(4), twistAccelSymbol(0), gridLocation(4, 1), color=a_color, fixed=True))

    for j in range(1, num_links+1):
        graph.addVaraible(Variable(angularAccelName(j), angularAccelSymbol(j), gridLocation(j - 0.5, 0), color=a_color))
        graph.addVaraible(Variable(torqueName(j), torqueSymbol(j), gridLocation(j - 0.5, 2.7), color=d_color))
        i1 = j - 1
        i2 = j
        i2_symbol = j % num_links
        graph.addVaraible(Variable(wrenchName(i1, j), wrenchSymbol(i1, j), gridLocation(j - 0.75, 2), color=d_color))
        graph.addVaraible(Variable(wrenchName(i2, j), wrenchSymbol(i2_symbol, j), gridLocation(j - 0.25, 2), color=d_color))

    for i in range(num_links):
        if i>0:
            j1 = i
            j2 = i + 1
            graph.addFactor(dynamicsFactorName(i), [twistAccelName(i), wrenchName(i, j1), wrenchName(i, j2)], gridLocation(i, 1.5), color=d_color)


    for j in range(1, num_links+1):
        i1 = j-1
        i2 = j
        graph.addFactor(accelFactorName(j), [twistAccelName(i1), twistAccelName(i2), angularAccelName(j)], gridLocation(j-0.5, 0.5), color=a_color)
        graph.addFactor(wrenchEqFactorName(j), [wrenchName(i1, j), wrenchName(i2, j)], gridLocation(j-0.5, 2), color=d_color)
        graph.addFactor(torqueFactorName(j), [torqueName(j), wrenchName(i2, j)], color=d_color)

    graph.generateLatexScript("simplified_four_bar.tikz")


def MarkovFG(n):
    graph = Graph()
    for i in range(n):
        graph.addVaraible(Variable(stateName(i), stateSymbol(i), gridLocation(i, 1)))
        if i<n-1:
            graph.addVaraible(Variable(actionName(i), actionSymbol(i), gridLocation(i+0.5, 1.5)))
        graph.addVaraible(Variable(observationName(i), observationSymbol(i), gridLocation(i, 2)))
    graph.addVaraible(Variable(ABName(), ABSymbol(), gridLocation((n-1)/2, 0)))
    graph.addVaraible(Variable(CName(), CSymbol(), gridLocation((n-1)/2, 3)))
    
    for i in range(n):
        graph.addFactor(observationFactorName(i), [stateName(i), observationName(i), CName()], gridLocation(i, 1.5))
        if i<n-1:
            graph.addFactor(transitionFactorName(i), [stateName(i), stateName(i+1), actionName(i), ABName()], gridLocation(i+0.5, 1))
    graph.generateLatexScript("Markov_FG.tikz")

def stateEstimationFG(n):
    graph = Graph()
    for i in range(n):
        graph.addVaraible(Variable(stateName(i), stateSymbol(i), gridLocation(i, 1)))
    
    for i in range(n):
        graph.addFactor(observationFactorName(i), [stateName(i)], gridLocation(i, 0.5))
        if i<n-1:
            graph.addFactor(transitionFactorName(i), [stateName(i), stateName(i+1)], gridLocation(i+0.5, 1))
    graph.generateLatexScript("estimation_FG.tikz")

def controlFG(n):
    graph = Graph()
    for i in range(n):
        graph.addVaraible(Variable(stateName(i), stateSymbol(i), gridLocation(i, 1)))
        if i<n-1:
            graph.addVaraible(Variable(actionName(i), actionSymbol(i), gridLocation(i+0.5, 1.5)))
    
    for i in range(n):
        graph.addFactor(stateCostFactorName(i), [stateName(i)], gridLocation(i, 0.5))
        if i<n-1:
            graph.addFactor(transitionFactorName(i), [stateName(i), stateName(i+1), actionName(i)], gridLocation(i+0.5, 1))
            graph.addFactor(actionCostFactorName(i), [actionName(i)], gridLocation(i+0.5, 2))
    graph.generateLatexScript("control_FG.tikz")

def SysIDFG(n):
    graph = Graph()
    for i in range(n):
        graph.addVaraible(Variable(stateName(i), stateSymbol(i), gridLocation(i, 1)))
    graph.addVaraible(Variable(ABName(), ABSymbol(), gridLocation((n-1)/2, 0)))
    graph.addVaraible(Variable(CName(), CSymbol(), gridLocation((n-1)/2, 2)))
    
    for i in range(n):
        graph.addFactor(observationFactorName(i), [stateName(i), CName()])
        if i<n-1:
            graph.addFactor(transitionFactorName(i), [stateName(i), stateName(i+1), ABName()], gridLocation(i+0.5, 1))
    graph.generateLatexScript("SysID_FG.tikz")

#################################### main function ####################################
def main():
    # serialFourBarFG()
    # simplifiedFourBarFG()
    # MarkovFG(4)
    # controlFG(4)
    # stateEstimationFG(4)
    # SysIDFG(4)
    jumpingRobotFG()

if __name__ == "__main__":
    main()