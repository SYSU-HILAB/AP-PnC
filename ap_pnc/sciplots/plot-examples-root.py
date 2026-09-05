
import matplotlib.pyplot as plt
import numpy as np
import pandas as pd
import matplotlib.gridspec as gridspec
from matplotlib.ticker import MultipleLocator

from examples.aero_data.aero_data_loader import AeroDataLoaderInstance



import sciplots

import os

print(plt.style.available)
# exit()
# based on this file dir 
current_dir = os.path.dirname(os.path.abspath(__file__))

figures_dir = os.path.join(current_dir, "figures")

if not os.path.exists(figures_dir):
    os.makedirs(figures_dir)


# chinese characters directly plotting with roman English
with plt.style.context(["ieee", "latex-sc"]):
    fig, ax = plt.subplots(figsize=(3.5, 2.5))
    ax.plot(range(5))

    ax.text(0.5, 3.,r"$\prescript{\mathcal{B}}{}{\boldsymbol{c}_z}$")
    
    # mathbf的字体会随着 text字体而改变
    ax.text(1.5, 3.,r"$\prescript{\mathcal{B}}{}{\mathbf{c}_z}$")
    ax.text(1.5, 2.,r"$\prescript{\mathcal{B}}{}{\boldsymbol{c}_z}$")
    
    ax.text(2.5, 3.,r"中国人不骗中国人")
    ax.text(2.5, 1.5,r"一眼顶真")
    ax.set_xlabel(r"中国人", loc="left")
    # set bottom large margin
    plt.subplots_adjust(bottom=0.15)
    plt.subplots_adjust(left=0.25)
    ax.set_ylabel(r"$\prescript{a}{b}{\sum_{i=1}^{n}} = \sum_{i=1}^{n} a_i b_i$")
    ax.set_title(r"$\boldsymbol{E} = \boldsymbol{mc}^2$")
    fig.savefig(os.path.join(figures_dir, "figure-chinese.pdf"), backend="pgf")


# chinese characters directly plotting with roman English
with plt.style.context(["ieee", "latex"]):
    fig, ax = plt.subplots(figsize=(3.5, 2.5))
    ax.plot(range(5))

    ax.text(0.5, 3.,r"$\prescript{\mathcal{B}}{}{\boldsymbol{c}_z}$")
    # mathbf的字体会随着 text字体而改变
    ax.text(1.5, 3.,r"$\prescript{\mathcal{B}}{}{\mathbf{c}_z}$")
    ax.text(1.5, 2.,r"$\prescript{\mathcal{B}}{}{\boldsymbol{c}_z}$")
    # set bottom large margin
    plt.subplots_adjust(bottom=0.15)
    plt.subplots_adjust(left=0.25)
    ax.set_ylabel(r"$\prescript{a}{b}{\sum_{i=1}^{n}} = \sum_{i=1}^{n} a_i b_i$")
    ax.set_title(r"$\boldsymbol{E} = \boldsymbol{mc}^2$")
    fig.savefig(os.path.join(figures_dir, "figure-english.pdf"), backend="pgf")



def initialize_figure():
    """Initialize a multi-panel figure with a grid layout."""
    fig_height = 3
    letter_column_width = 3.25
    fig = plt.figure(figsize=(letter_column_width, fig_height), constrained_layout=False)
    gs = gridspec.GridSpec(2, 1, figure=fig)
    gs.update(wspace=0.3, hspace=0.25)
    
    ax1 = fig.add_subplot(gs[0, 0])  # Cx plot
    ax2 = fig.add_subplot(gs[1, 0])  # Cz plot
    
    # Add more padding at the bottom for x-axis labels
    plt.subplots_adjust(bottom=0.15)
    plt.subplots_adjust(left=0.2)
    return fig, [ax1, ax2]

with plt.style.context(["ieee", "latex", "ticks-inward"]):
    fig, axs = initialize_figure()
    alpha_deg, cx, cz = AeroDataLoaderInstance.get_data()
    axs[0].plot(alpha_deg, cx)
    axs[1].plot(alpha_deg, cz)
    axs[1].set_xlabel(r"$\alpha~[\mathrm{deg}]$")
    axs[0].set_ylabel(r"$\prescript{\mathcal{B}}{}{\boldsymbol{c}_x}~[\mathrm{kg/m}]$")
    axs[1].set_ylabel(r"$\prescript{\mathcal{B}}{}{\boldsymbol{c}_z}~[\mathrm{kg/m}]$")
    # finetuning
    for ax in axs:
        sciplots.set_minor_ticks_style(ax)
    
    # set ticks points for first plot (cx)
    axs[0].set_yticks([-0.04, -0.02, 0, 0.02, 0.04])
    # Set minor ticks for y-axis (4 between each major tick)
    major_spacing = 0.02  # Distance between major ticks
    minor_spacing = major_spacing/5  # 4 minor ticks between majors
    axs[0].yaxis.set_minor_locator(MultipleLocator(minor_spacing))
    
        
    # seting x ticks lable false
    axs[0].set_xticklabels([])
    
            # set ticks points for second plot (cz)
    axs[1].set_yticks([-0.3, -0.15, 0, 0.15, 0.3])
    # Set minor ticks for y-axis (4 between each major tick)
    major_spacing = 0.15  # Distance between major ticks
    minor_spacing = major_spacing/5  # 4 minor ticks between majors
    axs[1].yaxis.set_minor_locator(MultipleLocator(minor_spacing))   
    
    fig.savefig(os.path.join(figures_dir, "figure-aero-lyu.pdf"), backend="pgf")



with plt.style.context(["ieee", "latex-sc", "ticks-inward"]):
    fig, axs = initialize_figure()
    alpha_deg, cx, cz = AeroDataLoaderInstance.get_data()
    axs[0].plot(alpha_deg, cx)
    axs[1].plot(alpha_deg, cz)
    axs[1].set_xlabel(r"攻角~[deg]")
    axs[0].set_ylabel(r"$\prescript{\mathcal{B}}{}{\boldsymbol{c}_x}~[\mathrm{kg/m}]$")
    axs[1].set_ylabel(r"$\prescript{\mathcal{B}}{}{\boldsymbol{c}_z}~[\mathrm{kg/m}]$")
    # finetuning
    for ax in axs:
        sciplots.set_minor_ticks_style(ax)
    
    # set ticks points for first plot (cx)
    axs[0].set_yticks([-0.04, -0.02, 0, 0.02, 0.04])
    # Set minor ticks for y-axis (4 between each major tick)
    major_spacing = 0.02  # Distance between major ticks
    minor_spacing = major_spacing/5  # 4 minor ticks between majors
    axs[0].yaxis.set_minor_locator(MultipleLocator(minor_spacing))
    
    # seting x ticks lable false
    axs[0].set_xticklabels([])
    
            # set ticks points for second plot (cz)
    axs[1].set_yticks([-0.3, -0.15, 0, 0.15, 0.3])
    # Set minor ticks for y-axis (4 between each major tick)
    major_spacing = 0.15  # Distance between major ticks
    minor_spacing = major_spacing/5  # 4 minor ticks between majors
    axs[1].yaxis.set_minor_locator(MultipleLocator(minor_spacing))   
    
    fig.savefig(os.path.join(figures_dir, "figure-aero-lyu-sc.pdf"), backend="pgf")


# 可以发现加入中文英文字体会稍微有一点不一样，但是mathfont还是基本一致




