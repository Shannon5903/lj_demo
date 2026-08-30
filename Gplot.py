import networkx as nx
import matplotlib.pyplot as plt 
import numpy as np
def plot_digraph(G, pos, title='G', rad=0.15, offset=0.8):
    # 设置中文支持
    plt.rcParams['font.sans-serif'] = ['SimHei']
    plt.rcParams['axes.unicode_minus'] = False
    # 绘制顶点
    nx.draw_networkx_nodes(G, pos, node_color='lightblue', node_size=500)
    nx.draw_networkx_labels(G, pos, font_size=12)
    # 分开画每条边，双向边用曲线
    for u, v in G.edges():
        if G.has_edge(v, u):
            # 双向边 → 曲线
            nx.draw_networkx_edges(
                G, pos, edgelist=[(u, v)],
                connectionstyle=f'arc3, rad={rad}',
                arrowstyle='-|>', arrowsize=12,
                edge_color='gray', alpha=0.6
            )
        else:
            # 单向边 → 直线
            nx.draw_networkx_edges(
                G, pos, edgelist=[(u, v)],
                arrowstyle='-|>', arrowsize=12,
                edge_color='gray', alpha=0.6
            )

    # 手动放置边权重标签，双向边的标签做偏移
    edge_labels = nx.get_edge_attributes(G, 'weight')
    for (u, v), weight in edge_labels.items():
        x1, y1 = pos[u]
        x2, y2 = pos[v]
        x_mid = (x1 + x2) / 2
        y_mid = (y1 + y2) / 2
        if G.has_edge(v, u):
            # 双向边：标签沿法线方向偏移，避免重叠
            dx = x2 - x1
            dy = y2 - y1
            length = np.hypot(dx, dy) + 1e-9
            nx_offset = -dy / length * rad * offset
            ny_offset = dx / length * rad * offset
            plt.text(x_mid + nx_offset, y_mid + ny_offset, str(weight),
                    ha='center', va='center', fontsize=8,
                    bbox=dict(boxstyle='round,pad=0.15', facecolor='white',
                            edgecolor='lightgray', alpha=0.85))
        else:
            # 单向边：标签放在中点
            plt.text(x_mid, y_mid, str(weight),
                    ha='center', va='center', fontsize=8,
                    bbox=dict(boxstyle='round,pad=0.15', facecolor='white',
                            edgecolor='lightgray', alpha=0.85))
    plt.axis('off')
    plt.title(title)
    plt.tight_layout()
    plt.show()
if __name__ == '__main__':
    N = 5
    weight_matrix = np.random.randint(1, 10, size=(N, N))
    print(weight_matrix)
    G = nx.complete_graph(N, create_using=nx.DiGraph())
    for i in range(N):
        for j in range(N):
            if i != j:
                G.edges[i, j]['weight'] = weight_matrix[i, j]
    pos = nx.shell_layout(G)
    plot_digraph(G, pos)