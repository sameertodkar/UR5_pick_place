import math
import pygame as pg
from pygame.math import Vector2


class Player(pg.sprite.Sprite):

    def __init__(self, pos=(420, 420)):
        super(Player, self).__init__()
        self.image = pg.Surface((70, 50), pg.SRCALPHA)
        pg.draw.polygon(self.image, (50, 120, 180), ((0, 0), (0, 50), (70, 25)))
        self.original_image = self.image
        self.rect = self.image.get_rect(center=pos)
        self.position = Vector2(pos)
        self.direction = Vector2(1, 0)  # A unit vector pointing rightward.
        self.speed = 2
        self.angle_speed = 0
        self.angle = 0

    def update(self):
        if self.angle_speed != 0:
            # Rotate the direction vector and then the image.
            self.direction.rotate_ip(self.angle_speed)
            self.angle += self.angle_speed
            self.image = pg.transform.rotate(self.original_image, -self.angle)
            self.rect = self.image.get_rect(center=self.rect.center)
        # Update the position vector and the rect.
        self.position += self.direction * self.speed
        self.rect.center = self.position


def main():
    pg.init()
    screen = pg.display.set_mode((1280, 720))
    player = Player((420, 420))
    playersprite = pg.sprite.RenderPlain((player))

    clock = pg.time.Clock()
    done = False
    while not done:
        clock.tick(60)
        for event in pg.event.get():
            if event.type == pg.QUIT:
                done = True
            elif event.type == pg.KEYDOWN:
                if event.key == pg.K_UP:
                    player.speed += 1
                elif event.key == pg.K_DOWN:
                    player.speed -= 1
                elif event.key == pg.K_LEFT:
                    player.angle_speed = -4
                elif event.key == pg.K_RIGHT:
                    player.angle_speed = 4
            elif event.type == pg.KEYUP:
                if event.key == pg.K_LEFT:
                    player.angle_speed = 0
                elif event.key == pg.K_RIGHT:
                    player.angle_speed = 0

        playersprite.update()

        screen.fill((30, 30, 30))
        playersprite.draw(screen)
        pg.display.flip()

if __name__ == '__main__':
    main()
    pg.quit()
