import React from 'react';
import Layout from '@theme/Layout';
import { SigninForm } from '../../components/auth/SigninForm';
import { useHistory } from '@docusaurus/router';

export default function SigninPage(): JSX.Element {
  const history = useHistory();

  const handleSigninSuccess = () => {
    // Redirect to home page after successful signin
    history.push('/');
  };

  return (
    <Layout
      title="Sign In"
      description="Sign in to your account"
    >
      <main>
        <SigninForm onSuccess={handleSigninSuccess} />
      </main>
    </Layout>
  );
}
