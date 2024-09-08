import networkx as nx
import random
import itertools
import time
from ortools.linear_solver import pywraplp

print("init")

def ler_instancia(caminho_arquivo):
    with open(caminho_arquivo, 'r') as f:
        # Ler a primeira linha (n, nc, m)
        primeira_linha = f.readline().strip().split()
        n = int(primeira_linha[0])
        nc = int(primeira_linha[1])
        m = int(primeira_linha[2])
        
        # Inicializar o grafo
        G = nx.Graph()
        
        # Ler os vértices centrais e seus graus mínimos
        centrais = {}
        for _ in range(nc):
            linha = f.readline().strip().split()
            vertice_central = int(linha[0])
            grau_minimo = int(linha[1])
            centrais[vertice_central] = grau_minimo
        
        # Ler as arestas e os custos
        for _ in range(m):
            linha = f.readline().strip().split()
            i = int(linha[0])
            j = int(linha[1])
            custo = int(linha[2])
            G.add_edge(i, j, weight=custo)
    
    return G, centrais

def eliminacao_ciclos(G, centrais): 
    # Criando o solver
    solver = pywraplp.Solver.CreateSolver('SCIP')

    if not solver:
        return "Solver não disponível." 

    # Variáveis de decisão: 1 se a aresta for selecionada, 0 caso contrário
    x = {}
    for u, v in G.edges():
        x[(u, v)] = solver.BoolVar(f'x_{u}_{v}')
        x[(v, u)] = solver.BoolVar(f'x_{v}_{u}')  # Para garantir a simetria das arestas (grafo não direcionado)

    # Função objetivo: minimizar o custo total das arestas selecionadas
    solver.Minimize(solver.Sum(G.edges()[u,v]["weight"] * x[(u, v)] for u, v in G.edges()))

    # Eliminação de ciclos (Subtour Elimination Constraints - SECs)
    # Para cada subconjunto de nós, garantir que o número de arestas seja <= |S| - 1
    for subset in itertools.chain.from_iterable(itertools.combinations(G.nodes(), r) for r in range(2, len(G.nodes()))):
        subset = set(subset)
        if len(subset) > 1:
            solver.Add(solver.Sum(x[(u, v)] for u, v in itertools.combinations(subset, 2) if G.has_edge(u, v)) <= len(subset) - 1)

    # Restrições de grau mínimo para os vértices centrais
    terminais = set(G.nodes()) - set(centrais.keys())
    for terminal in terminais:
        solver.Add(solver.Sum(x[terminal,v] for v in G.neighbors(terminal)) == 1)

    # Definir limite de tempo de execução
    solver.SetTimeLimit(1800000)  # 30 minutos em milissegundos

    # Resolver o problema
    start_time = time.time()
    status = solver.Solve()
    final_time = time.time() - start_time

    if status == pywraplp.Solver.OPTIMAL:
        print("Solução ótima encontrada.")
    else:
        print("Solução não encontrada ou tempo limite excedido.")
    
    return final_time

def rotulos_vertices(G, centrais):
    solver = pywraplp.Solver.CreateSolver('SCIP')

    if not solver:
        return "Solver não disponível." 

    # Variáveis de decisão: 1 se o arco (i, j) estiver na árvore, 0 caso contrário
    y = {}
    for u, v in G.edges():
        y[(u, v)] = solver.BoolVar(f'y_{u}_{v}')  # Considerando o arco direcionado
        y[(v, u)] = solver.BoolVar(f'y_{v}_{u}')  # Para garantir a simetria das arestas

    # Função objetivo: minimizar o custo total das arestas selecionadas
    solver.Minimize(solver.Sum(G[u][v]['weight'] * y[(u, v)] for u, v in G.edges()))

    # Escolher um vértice raiz arbitrário
    root = 0  

    # Variáveis de rótulo nos vértices
    labels = {}
    n = len(G.nodes())
    for v in G.nodes():
        labels[v] = solver.IntVar(1, n, f'label_{v}')
    
    # Restrições de rótulos (MTZ)
    for u, v in G.edges():
        solver.Add(labels[u] - labels[v] + n * y[(u, v)] <= n - 1)

    # Restrição de árvore geradora: cada nó (exceto a raiz) deve ter exatamente uma aresta conectada
    for v in G.nodes():
        if v != root:
            solver.Add(solver.Sum(y.get((u, v), 0) for u in G.neighbors(v)) == 1)

    # Restrições de grau mínimo para os vértices centrais
    for v, grau_minimo in centrais.items():
        solver.Add(solver.Sum(y.get((u, v), 0) + y.get((v, u), 0) for u in G.neighbors(v)) >= grau_minimo)

    # Definir limite de tempo de execução
    solver.SetTimeLimit(1800000)  # 30 minutos em milissegundos

    # Resolver o problema
    start_time = time.time()
    status = solver.Solve()
    end_time = time.time()

    if status == pywraplp.Solver.OPTIMAL:
        print("Solução ótima encontrada.")
        return solver.Objective().Value(), end_time - start_time
    else:
        print("Solução não encontrada ou tempo limite excedido.")
        return None, end_time - start_time



# Função para comparação entre as formulações
def comparar_formulacoes(caminho_arquivo):
    # Ler a instância
    G, centrais = ler_instancia(caminho_arquivo)
    
    print(f"Comparando formulações para instância {caminho_arquivo}")
    
    # Eliminação de Ciclos
    tempo_eliminacao_ciclos = eliminacao_ciclos(G, centrais)
    print(f"Tempo Eliminação de Ciclos: {tempo_eliminacao_ciclos:.2f} segundos")
    

# Exemplo de uso com as instâncias
instancias = [
    # "instancias/tb8ch4_0.txt",
    # "instancias/tb8ch4_1.txt",
    "instancias/tb8ch8_0.txt",
    # "instancias/tb8ch8_1.txt",
    # "instancias/tb8ch10_0.txt",
    # "instancias/tb8ch10_1.txt",
    # "instancias/tb8ch15_0.txt",
    # "instancias/tb8ch15_1.txt",
    # "instancias/tb8ch20_0.txt",
    # "instancias/tb8ch25_0.txt",
    # "instancias/tb8ch25_1.txt",
    # "instancias/tb8ch30_0.txt",
    # "instancias/tb8ch30_1.txt"
]

for instancia in instancias:
    comparar_formulacoes(instancia)
