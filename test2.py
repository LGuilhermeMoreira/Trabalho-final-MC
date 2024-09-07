import networkx as nx
import random
import time

# Gerar um grafo aleatório com pesos nas arestas
def generate_graph(n, p):
    G = nx.gnp_random_graph(n, p)
    for (u, v) in G.edges():
        G.edges[u, v]['weight'] = random.randint(10, 500)
    return G

# Eliminação de Ciclos usando a Árvore Geradora Mínima
def elimination_of_cycles(G):
    # Gera a Árvore Geradora Mínima
    mst = nx.minimum_spanning_tree(G, weight='weight')
    return mst

# Rótulos nos Vértices (baseado na formulação MTZ)
def vertex_labels(G):
    # Transformar o grafo em direcionado para a formulação de rótulos
    D = nx.DiGraph()
    for (u, v, data) in G.edges(data=True):
        weight = data['weight']
        D.add_edge(u, v, weight=weight)
        D.add_edge(v, u, weight=weight)

    # Escolhe um vértice raiz arbitrário
    root = list(G.nodes())[0]

    # Atribui rótulos para garantir que a ordem seja respeitada
    labels = {node: float('inf') for node in G.nodes()}
    labels[root] = 0

    # Aplica restrições de rótulos (MTZ) para eliminar ciclos
    for (u, v) in D.edges():
        if labels[u] == float('inf'):
            labels[u] = labels[v] + 1
        elif labels[v] == float('inf'):
            labels[v] = labels[u] + 1
    
    # Constrói a árvore com base nos rótulos
    tree = nx.DiGraph()
    for (u, v) in D.edges():
        if labels[u] < labels[v]:
            tree.add_edge(u, v, weight=D.edges[u, v]['weight'])

    return tree

# Função para comparar as formulações
def compare_formulations(n, p):
    G = generate_graph(n, p)
    
    # Eliminação de Ciclos
    start_time = time.time()
    mst1 = elimination_of_cycles(G)
    time_ec = time.time() - start_time
    
    # Rótulos nos Vértices
    start_time = time.time()
    mst2 = vertex_labels(G)
    time_vl = time.time() - start_time
    
    return time_ec, time_vl, mst1, mst2

# Comparação para diferentes valores de n e p
def run_comparisons():
    for n in range(10, 51, 10):
        for p in [0.1, 0.3, 0.5]:
            print(f"n = {n}, p = {p}")
            time_ec, time_vl, mst1, mst2 = compare_formulations(n, p)
            print(f"  Eliminação de Ciclos: {time_ec:.4f}s")
            print(f"  Rótulos nos Vértices: {time_vl:.4f}s")
            print('-' * 40)

if __name__ == "__main__":
    run_comparisons()
