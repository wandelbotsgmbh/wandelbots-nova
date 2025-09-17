import {
  type CardContentProps,
  CardHeader,
  type CardHeaderProps,
  type CardProps,
  Card as MuiCard,
  CardContent as MuiCardContent,
} from '@mui/material'
import React, { type ReactNode } from 'react'

export interface SectionCardProps extends Omit<CardProps, 'title'> {
  title?: ReactNode
  subheader?: ReactNode
  contentProps?: CardContentProps
  headerProps?: Partial<CardHeaderProps>
  children?: ReactNode
}

const SectionCard: React.FC<SectionCardProps> = ({
  title,
  subheader,
  contentProps,
  headerProps,
  children,
  ...cardProps
}) => {
  return (
    <MuiCard
      {...cardProps}
      sx={{ borderRadius: 4, backgroundColor: 'transparent' }}
    >
      {(title || subheader) && (
        <CardHeader title={title} subheader={subheader} {...headerProps} />
      )}
      <MuiCardContent {...contentProps}>{children}</MuiCardContent>
    </MuiCard>
  )
}

export default SectionCard
export { SectionCard }
