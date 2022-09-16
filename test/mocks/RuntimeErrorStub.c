#include "RuntimeErrorStub.h"

static const char * message = "No Error";
static int parameter = -1;
static const char * file = 0;
static int line  = -1;

void RunTimeErrorStub_Reset(void)
{
    message = "No Error";
    parameter = -1;
}

const char * RunTimeErrorStub_GetLastError(void)
{
    return message;
}

void RunTimeError(const char * m, int p, const char * f, int l)
{
    message = m;
    parameter = p;
    file = f;
    line = l;
}

int RunTimeErrorStub_GetLastParameter(void)
{
    return parameter;
}