//
// Created by araceli on 29/10/18.
//

#ifndef PROJECT_BODYVISUALIZER_H
#define PROJECT_BODYVISUALIZER_H

#include <QDebug>
#include <SFML/Graphics.hpp>
#include <astra/astra.hpp>
#include <iostream>
#include <cstring>
#include <sfLine.h>

#endif //PROJECT_BODYVISUALIZER_H
class BodyVisualizer : public astra::FrameListener
{
public:


    BodyVisualizer();
    ~BodyVisualizer();

    virtual void on_frame_ready(astra::StreamReader& reader,
                                astra::Frame& frame) override;
    static  sf::Color get_body_color(std::uint8_t bodyId);
    void init_depth_texture(int width, int height);
    void init_overlay_texture(int width, int height);
    void check_fps();
    void processDepth(astra::Frame& frame);
    void processBodies(astra::Frame& frame);
    void update_body(astra::Body body, const float jointScale);
    void update_bone(const astra::JointList& joints,
                     const float jointScale,astra::JointType j1,
                     astra::JointType j2);
    void update_overlay(const astra::BodyMask& bodyMask,
                        const astra::FloorMask& floorMask);
    void clear_overlay();
    void draw_bodies(sf::RenderWindow& window);
    void draw_to(sf::RenderWindow& window);



private:
    long double frameDuration_{ 0 };
    std::clock_t lastTimepoint_ { 0 };
    sf::Texture texture_;
    sf::Sprite sprite_;

    using BufferPtr = std::unique_ptr < uint8_t[] >;
    BufferPtr displayBuffer_{ nullptr };

    std::vector<astra::Vector2f> jointPositions_;

    int depthWidth_{0};
    int depthHeight_{0};
    int overlayWidth_{0};
    int overlayHeight_{0};

    std::vector<sfLine> boneLines_;
    std::vector<sfLine> boneShadows_;
    std::vector<sf::CircleShape> circles_;
    std::vector<sf::CircleShape> circleShadows_;

    float lineThickness_{ 0.5f }; // pixels
    float jointRadius_{ 1.0f };   // pixels
    float shadowRadius_{ 0.5f };  // pixels

    BufferPtr overlayBuffer_{ nullptr };
    sf::Texture overlayTexture_;
    sf::Sprite overlaySprite_;

};