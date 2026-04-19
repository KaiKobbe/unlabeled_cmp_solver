import matplotlib.pyplot as plt
import matplotlib.backends.backend_pdf as pdf_backend
import networkx as nx
from matplotlib.lines import Line2D

from .schedule import Schedule

def visualize_schedule(instance, schedule: Schedule, path: str | None = None, k: int | None = None) -> None:
    """
    Simple visualization function for schedules.
    Outputs a PDF at the specified path showing the robot positions at each timestep.
    If path is None, the frames are displayed inline (e.g. in a Jupyter notebook).
    If the graph has node position attributes, they are used for the layout; otherwise
    a spring layout is computed automatically.
    The optional parameter k limits the output to the first k timesteps.
    """
    graph = instance.graph
    makespan = schedule.makespan
    timesteps = range(min(k, makespan + 1) if k is not None else makespan + 1)

    if all("pos" in graph.nodes[v] for v in graph.nodes):
        pos = {v: graph.nodes[v]["pos"] for v in graph.nodes}
    else:
        pos = nx.spring_layout(graph, seed=42)

    nodes = list(graph.nodes)
    target_set = set(instance.target_positions)
    robot_paths = schedule.robot_paths

    def make_frame(t: int):
        fig, ax = plt.subplots(figsize=(8, 6))
        ax.set_title(f"Timestep {t} / {makespan}", fontsize=14, fontweight="bold")
        ax.axis("off")
        robot_nodes = {robot_paths[r][t] for r in range(schedule.num_robots)}
        non_robot_nodes = [v for v in nodes if v not in robot_nodes]
        nx.draw_networkx_edges(graph, pos, ax=ax, alpha=0.8, width=2.5)
        nx.draw_networkx_nodes(graph, pos, nodelist=list(robot_nodes),
                               node_color="tomato", node_size=600, ax=ax)
        nx.draw_networkx_nodes(graph, pos, ax=ax,
                               nodelist=[v for v in non_robot_nodes if v not in target_set],
                               node_color="black", node_size=130)
        nx.draw_networkx_nodes(graph, pos, ax=ax,
                               nodelist=[v for v in non_robot_nodes if v in target_set],
                               node_color="black", node_size=130, node_shape="X")
        nx.draw_networkx_nodes(graph, pos, ax=ax,
                               nodelist=[v for v in robot_nodes if v not in target_set],
                               node_color="black", node_size=130)
        nx.draw_networkx_nodes(graph, pos, ax=ax,
                               nodelist=[v for v in robot_nodes if v in target_set],
                               node_color="black", node_size=130, node_shape="X")
        legend_elements = [
            Line2D([0], [0], marker="X", color="w", markerfacecolor="black",
                markersize=10, label="Target"),
            Line2D([0], [0], marker="o", color="w", markerfacecolor="tomato",
                markersize=13, label="Robot"),
        ]
        ax.legend(
            handles=legend_elements,
            loc="upper center",
            bbox_to_anchor=(0.5, -0.02),
            ncol=2,
            framealpha=0.0,
            fontsize=11,
        )
        return fig

    if path is not None:
        plt.ioff()
        with pdf_backend.PdfPages(path) as pdf:
            for t in timesteps:
                fig = make_frame(t)
                pdf.savefig(fig)
                plt.close(fig)
        plt.ion()
    else:
        for t in timesteps:
            fig = make_frame(t)
            plt.show()
            plt.close(fig)