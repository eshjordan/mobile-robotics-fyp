import pygame
# import math

from math import pi, sin, cos

import colorsys

from algos import *
import numpy as np

from time import sleep

# Initialize Pygame
pygame.init()

# Define constants
WIDTH, HEIGHT = 960, 960
WHITE = (255, 255, 255, 255)
BLACK = (0, 0, 0, 255)
GREEN = (50, 255, 0, 255)
GREEN_TRANS = (50, 255, 0, 100)
RED = (255, 0, 0, 255)
RADIUS = 380  # Radius of the circle
ALPHA = 0.5

# Set up the display
screen = pygame.display.set_mode((WIDTH, HEIGHT))

pygame.font.init()

# Choose a font and size
font_size = 30
font = pygame.font.SysFont(None, font_size)  # Default font

# Choose a color (R, G, B)
font_color = (255, 255, 255)  # White

def hue_to_rgb(hue, alpha):
    """
    Convert a hue value (0-360 degrees) to an RGB color.
    
    :param hue: Hue value (0-360 degrees)
    :return: Tuple representing the RGB color (R, G, B)
    """
    hue_normalized = hue / 360.0  # Normalize hue to range [0, 1]
    hue_normalized = 0.1*hue_normalized + 0
    r, g, b = colorsys.hsv_to_rgb(hue_normalized, 1.0, 1.0)  # Full saturation and value
    return (int(r * 255), int(g * 255), int(b * 255), int(alpha * 255))

def generate_wedge(center, radius, angle, n=20):

    vertices = [center]
    # for p in [angle+dom_rad*i/n for i in range(-n,n+1)]:
    # for p in range(angle[0]%(2*pi), angle[1]%(2*pi), step):
    a0, a1 = angle
    if a1 < a0:
        a1 += 2*pi
    for p in np.linspace(a0, a1, n):
        vertices.append((center[0]-radius*cos(p), center[1]-radius*sin(p)))

    return vertices

# Function to draw wedges
def draw_wedges(screen, center, radius, agent_thetas, highlighted):

    polygons = [generate_wedge(center, radius, angle) for angle in agent_thetas]
    cols = [hue_to_rgb(int((180/pi)*(pi+angle[0])),ALPHA) for angle in agent_thetas]

    # Draw wedges
    for i in range(len(polygons)):
        transparent_surface = pygame.Surface((WIDTH, HEIGHT), pygame.SRCALPHA)
        pygame.draw.polygon(transparent_surface, cols[i], polygons[i], 0)
        screen.blit(transparent_surface, (0, 0))
    
    for i in highlighted:
        transparent_surface = pygame.Surface((WIDTH, HEIGHT), pygame.SRCALPHA)
        pygame.draw.polygon(transparent_surface, GREEN_TRANS, polygons[i], 0)
        screen.blit(transparent_surface, (0, 0))
        pygame.draw.polygon(screen, GREEN, polygons[i], 2)

def label_points(screen, center, radius, agents, highlighted):

    for i,agent in enumerate(agents):

        # Circle coords
        x1 = center[0]-radius*cos(agent.theta_c())
        y1 = center[1]-radius*sin(agent.theta_c())
        # Text coords
        # x2 = x1 - 20
        # y2 = y1- 20
        a = 1.12
        x2 = center[0]-(radius*a)*cos(agent.theta_c()) - 20
        y2 = center[1]-(radius*a)*sin(agent.theta_c()) - 10
        # Draw circle
        HL = (i in highlighted)
        colour = GREEN if HL else WHITE
        rad = 10 if HL else 7
        pygame.draw.circle(screen, colour, (x1,y1), rad)
        # Draw number
        # number_text = f"{i+1} (n={agent.n})"
        number_text = f"{agent.n} ({i+1})"
        text_surface = font.render(number_text, True, GREEN if HL else font_color)
        screen.blit(text_surface, (x2,y2))



def draw_update(screen, center, agents, highlighted=[]):

        agent_thetas = [(a.theta_l, a.theta_u) for a in agents]

        # Fill the background
        screen.fill(BLACK)

        # Draw the circle
        pygame.draw.circle(screen, WHITE, center, RADIUS, 4)

        # Draw the wedges
        draw_wedges(screen, center, RADIUS-2, agent_thetas, highlighted)

        label_points(screen, center, RADIUS, agents, highlighted)

        # Update the display
        pygame.display.flip()

import sys
def main():

    NUM_AGENTS = 7

    if len(sys.argv) >= 2:
        NUM_AGENTS = int(sys.argv[1])

    print(f"NUM_AGENTS = {NUM_AGENTS}")
    pygame.display.set_caption(f"NUM_AGENTS = {NUM_AGENTS}")


    running = True
    center = (WIDTH // 2, HEIGHT // 2)

    agents = sequential_add_int(NUM_AGENTS)
    for agent in agents:
        print(agent)
    
    draw_update(screen, center, agents)
    
    # dominance_rads = [(a.theta_c() - a.theta_l)%(2*pi) for a in agents]

    p = pairwise_interaction(agents, lambda highlighted : draw_update(screen, center, agents, highlighted))
    print("done 4")

    naive_extension(agents, lambda highlighted : draw_update(screen, center, agents, highlighted), p)
    print("done 6")

    # Finish loop
    while running:

        sleep(1)

        for event in pygame.event.get():
            if event.type == pygame.QUIT:
                running = False
        
    pygame.quit()




if __name__ == "__main__":
    main()
