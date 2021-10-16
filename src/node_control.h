#pragma once

class Node_Control
{
public:
    void init();
    void shutdown();
private:
    static Node_Control * m_this;
};