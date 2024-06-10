#ifndef __cmd_parser_hpp__
#define __cmd_parser_hpp__

class LineParsingResult {
    private:
        const unsigned int maxTokens;
        const unsigned int maxLengthPerToken;
        bool ok;
        char** result;
        unsigned int next;
    public:
        LineParsingResult(int maxTokens, int maxLengthPerToken);
        bool addToken(char *);
        int getTokenCount();
        char* getToken(int i);
        void reset();
};

class LineParser {
    private:
        const unsigned int capacity;
        char* buffer;
        unsigned int next;
    public:
        LineParser(int capacity);
        bool process(char*, char separator, LineParsingResult&);
};

#endif
