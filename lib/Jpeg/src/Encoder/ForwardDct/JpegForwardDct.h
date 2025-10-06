#pragma once

namespace Jpeg
{

class EncoderBuffer;
class Quantizer;

class ForwardDct
{
public:
  static void perform(EncoderBuffer& buffer, const Quantizer* quantizers, const int* componentQuantizerIndices);
};

}
