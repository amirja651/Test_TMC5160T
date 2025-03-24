class SystemManager {
public:
    static void initialize();
    static void handleFatalError(const char* errorMsg);
    static void updateStatus();
    static bool isSystemHealthy();
};