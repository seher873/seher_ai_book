// Utility functions for authentication
import { useAuth } from '../auth-client';

// Custom hook to access authentication status
export const useAuthStatus = () => {
  const { session, isPending } = useAuth();

  return {
    isAuthenticated: !!session,
    user: session?.user,
    loading: isPending,
    error: null
  };
};

// Higher-order component to wrap pages that require authentication
export const withAuth = (Component) => {
  return function AuthenticatedComponent(props) {
    const { isAuthenticated, loading, user } = useAuthStatus();

    if (loading) {
      return (
        <div style={{
          display: 'flex',
          justifyContent: 'center',
          alignItems: 'center',
          height: '100vh',
          fontSize: '18px'
        }}>
          Loading...
        </div>
      );
    }

    if (!isAuthenticated) {
      // Redirect to login
      window.location.href = '/auth/login';
      return null;
    }

    return <Component {...props} user={user} />;
  };
};