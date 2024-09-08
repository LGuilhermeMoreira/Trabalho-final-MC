from ortools.linear_solver import pywraplp
import networkx as nx
import random

def eliminacao_de_ciclos(graph, centrais, grau_minimo):
    solver = pywraplp.Solver.CreateSolver('SCIP')
    
    # Variáveis binárias para as arestas
    x = {}
    for u, v in graph.edges():
        # Arestas são armazenadas de forma ordenada (u < v) para evitar problemas de chave
        x[(min(u, v), max(u, v))] = solver.BoolVar(f"x_{min(u, v)}_{max(u, v)}")

    # Função objetivo: minimizar o custo total da árvore geradora
    solver.Minimize(solver.Sum(graph[u][v]['weight'] * x[(min(u, v), max(u, v))] for u, v in graph.edges()))

    # Restrição 1: A soma das arestas entre os vértices centrais deve ser c-1
    solver.Add(solver.Sum(x[(min(u, v), max(u, v))] for u, v in graph.edges() if u in centrais and v in centrais) == len(centrais) - 1)

    # Restrição 2: Eliminar ciclos para os vértices centrais
    for subset in nx.find_cliques(graph.subgraph(centrais)):
        if len(subset) > 1:
            solver.Add(solver.Sum(x[(min(u, v), max(u, v))] for u, v in graph.edges(subset) if (min(u, v), max(u, v)) in x) <= len(subset) - 1)
    
    # Restrição 3: Grau mínimo dos vértices centrais
    for v in centrais:
        solver.Add(solver.Sum(x[(min(u, v), max(u, v))] for u, v in graph.edges(v) if (min(u, v), max(u, v)) in x) >= grau_minimo)
    
    # Resolvendo o problema
    status = solver.Solve()

    if status == pywraplp.Solver.OPTIMAL:
        print('Solução ótima encontrada!')
        arvore = [(u, v) for u, v in graph.edges() if x[(min(u, v), max(u, v))].solution_value() > 0.5]
        return arvore
    else:
        print('Nenhuma solução ótima encontrada dentro do tempo limite.')
        return None


def rotulacao_de_vertices(graph, centrais, grau_minimo):
    solver = pywraplp.Solver.CreateSolver('SCIP')

    # Variáveis binárias para as arestas
    y = {}
    for u, v in graph.edges():
        # Arestas são armazenadas de forma ordenada (u < v) para evitar problemas de chave
        y[(min(u, v), max(u, v))] = solver.BoolVar(f"y_{min(u, v)}_{max(u, v)}")
    
    # Variáveis de rótulos para os vértices centrais
    r = {}
    for v in centrais:
        r[v] = solver.IntVar(1, len(centrais), f"r_{v}")

    # Função objetivo: minimizar o custo total da árvore geradora
    solver.Minimize(solver.Sum(graph[u][v]['weight'] * y[(min(u, v), max(u, v))] for u, v in graph.edges()))

    # Restrição 1: Cada vértice (exceto a raiz) deve ter uma aresta de entrada
    raiz = centrais[0]
    for v in centrais:
        if v != raiz:
            solver.Add(solver.Sum(y[(min(u, v), max(u, v))] for u, v in graph.edges(v) if (min(u, v), max(u, v)) in y) == 1)

    # Restrição 2: Grau mínimo dos vértices centrais
    for v in centrais:
        solver.Add(solver.Sum(y[(min(u, v), max(u, v))] for u, v in graph.edges(v) if (min(u, v), max(u, v)) in y) >= grau_minimo - 1)
    
    # Restrição 3: Condições de rótulos para evitar ciclos (MTZ)
    for u, v in graph.edges():
        if u in centrais and v in centrais:
            solver.Add(r[u] - r[v] + (len(centrais)) * y[(min(u, v), max(u, v))] <= len(centrais) - 1)

    # Resolvendo o problema
    status = solver.Solve()

    if status == pywraplp.Solver.OPTIMAL:
        print('Solução ótima encontrada!')
        arvore = [(u, v) for u, v in graph.edges() if y[(min(u, v), max(u, v))].solution_value() > 0.5]
        return arvore
    else:
        print('Nenhuma solução ótima encontrada dentro do tempo limite.')
        return None


graph = nx.gnp_random_graph(10, 0.5)
for u, v in graph.edges():
    graph[u][v]['weight'] = random.randint(10, 500)


centrais = list(range(5))  # Supondo que os primeiros 5 vértices são centrais
grau_minimo = 2

# Eliminação de ciclos
arvore_ciclos = eliminacao_de_ciclos(graph, centrais, grau_minimo)

# Rotulação de vértices
arvore_rotulos = rotulacao_de_vertices(graph, centrais, grau_minimo)
