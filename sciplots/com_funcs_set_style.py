default_ieee_colors = {
    "red": "#B43734",
    "blue": "#00159D",
    "cyan": "#397E77",
    "yellow": "#E9CC49"
}

colors = default_ieee_colors



def set_minor_ticks_style(ax):
    ax.grid(True, which='minor', linestyle=':', alpha=0.4)
    
    
def set_label_padding_both(ax, padding):
    ax.set_xlabel(ax.get_xlabel(), labelpad=padding)
    ax.set_ylabel(ax.get_ylabel(), labelpad=padding)
    
def set_label_padding_x(ax, padding):
    ax.set_xlabel(ax.get_xlabel(), labelpad=padding)
    
def set_label_padding_y(ax, padding):
    ax.set_ylabel(ax.get_ylabel(), labelpad=padding)